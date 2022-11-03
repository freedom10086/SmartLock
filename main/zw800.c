#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "zw800.h"
#include "common.h"

#define TAG "zw800"

#define BUF_SIZE (512)
#define READ_RESPONSE_TIME_OUT 400

/**
 * pin 1-6
 * 1. V_TOUCH 3.3始终供电
 * 2. TouchOut 高电平有触摸 休眠后恢复低电平
 * 3. VCC
 * 4. Tx   波特率57600
 * 5. Rx
 * 6. GNd
 */
typedef struct {
    uint8_t *buff;
    uart_port_t uart_port;
    int touch_pin;
    TaskHandle_t tsk_hdl;
} zw800_t;

/**
 * 命令包：
 * 包头(2)   芯片地址(4)  包标识(1) 包长度(2)   指令(1) 参数 1 … 参数 N 校验和(2)
 * 0xEF01  0XFF*4       0X01        X
 * 包长度x等于 指令（1） + 参数 + 校验和（2）
 *
 * 应答包：
 * 包头(2)   芯片地址(4)  包标识(1) 包长度(2)   确认码(1) 参数 1 … 参数 N 校验和(2)
 * 确认码：
 * 00H：表示指令执行完毕或 OK；
 * 01H：表示数据包接收错误；
 * 02H：表示传感器上没有手指；
 * 09H：表示没搜索到指纹；
 * 0aH：表示特征合并失败；
 * 0bH：表示访问指纹库时地址序号超出指纹库范围；
 * 10H：表示删除模板失败；
 * 11H：表示清空指纹库失败；
 * 13H：表示口令不正确；
 * 1eH：自动注册（enroll）失败；
 * 1fH：指纹库满；
 */
typedef struct {
    uint8_t data[20];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} zw800_cmd_t;

uint8_t write_buf[30];
zw800_cmd_t ZW800_CMD_COMMON_HEAD = {{0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01}, 7};
zw800_cmd_t ZW800_CMD_SLEEP = {{0x33,}, 1};
// 检测模组是否正常工作
zw800_cmd_t ZW800_CMD_ECHO = {{0x35,}, 1};
zw800_cmd_t ZW800_CMD_GET_IMAGE = {{0x01,}, 1}; // 验证用获取图像 =01H 表示收包有错, 02H 表示传感器上无手指
// 将获取到的原始图像生成指纹特征, 存到 参数1索引的charBuff CharBuffer1、CharBuffer2 、CharBuffer3，CharBuffer4,
zw800_cmd_t ZW800_CMD_PS_GetChar = {{0x02, 0x01}, 2};
// 以特征文件缓冲区中的特征文件搜索整个或部分指纹库             /* buffId,  startPage          pageNum */ (0 - 100)
zw800_cmd_t ZW800_CMD_PS_SearchM = {{0x04, 0x01, 0x00, 0x00, 0x00, 0x64}, 6};
zw800_cmd_t ZW800_CMD_PS_EMPTY = {{0x0D}, 1}; // 清空指纹库
zw800_cmd_t ZW800_CMD_RESTORE_TO_DEFAULT = {{0x3B}, 1}; // 恢复出厂设置
// 读有效模板个数
zw800_cmd_t ZW800_CMD_PS_ValidTempleteNum = {{0x1D,}, 1};
// 自动注册 id号(2) 录入次数(1) 参数(2)
zw800_cmd_t ZW800_CMD_PS_AutoEnroll = {{0x31, '\0', '\0', 0x04, '\0', '\0'}, 6};
// 自动验证 分数等级(2) 1-5 默认3 id号(2) 0xFFFF 全搜索 参数(2)
zw800_cmd_t ZW800_CMD_PS_AutoIdentify = {{0x32, 0x03, 0xff, 0xff, '\0', '\0'}, 6};
// LED 功能码(1) 起始颜色(1) 结束颜色(1) 循环次数(1)
// bit0 是蓝灯控制位；bit1 是绿灯控制位；bit2是红灯控制位
zw800_cmd_t ZW800_CMD_PS_CONTROL_LED = {{0x3C, 0x01, 0x07, 0x07, 0x00}, 5};

static zw800_t zw800;
static QueueHandle_t event_queue;

int control_led(zw800_t *zw800_dev, bool r, bool g, bool b, uint8_t repeat);

static void zw800_write_cmd(zw800_t *zw800_dev, zw800_cmd_t cmd) {
    uint16_t pack_len = cmd.databytes + 2;
    uint8_t len = ZW800_CMD_COMMON_HEAD.databytes + 2 + cmd.databytes + 2;

    // head
    memcpy(write_buf, ZW800_CMD_COMMON_HEAD.data, ZW800_CMD_COMMON_HEAD.databytes);
    // len
    write_buf[ZW800_CMD_COMMON_HEAD.databytes] = (pack_len >> 8) & 0xFF;
    write_buf[ZW800_CMD_COMMON_HEAD.databytes + 1] = pack_len & 0xFF;

    // cmd and param
    memcpy(&write_buf[ZW800_CMD_COMMON_HEAD.databytes + 2], cmd.data, cmd.databytes);

    // checksum
    uint16_t check_sum = 0x01; // 包标识为0x01
    for (int i = 0; i < 2 + cmd.databytes; ++i) {
        check_sum += write_buf[ZW800_CMD_COMMON_HEAD.databytes + i];
    }
    write_buf[ZW800_CMD_COMMON_HEAD.databytes + 2 + cmd.databytes] = (check_sum >> 8) & 0xFF;
    write_buf[ZW800_CMD_COMMON_HEAD.databytes + 2 + cmd.databytes + 1] = check_sum & 0xFF;

    uart_write_bytes(zw800_dev->uart_port, write_buf, len);

    ESP_LOGI(TAG, "write cmd to zw800 ");
            esp_log_buffer_hex(TAG, write_buf, len);

    vTaskDelay(pdMS_TO_TICKS(5));
}

static uint8_t zw800_read_response(zw800_t *zw800_dev) {
    int len = uart_read_bytes(zw800_dev->uart_port, zw800_dev->buff, (BUF_SIZE - 1),
                              READ_RESPONSE_TIME_OUT / portTICK_PERIOD_MS);
    if (len > 0 && len < 11) {
        ESP_LOGE(TAG, "invalid read len %d", len);
        return 0xFF;
    }

    if (len > 0) {
        ESP_LOGI(TAG, "read len %d :", len);
                esp_log_buffer_hex(TAG, zw800_dev->buff, len);
        if (len >= 12 && zw800_dev->buff[6] == 0x07) {
            return zw800_dev->buff[9];
        }
    }
    return 0xFF;
}

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(event_queue, &gpio_num, NULL);
}

int get_valid_template_num(zw800_t *zw800_dev) {
    zw800_write_cmd(zw800_dev, ZW800_CMD_PS_ValidTempleteNum);
    if (0x00 == zw800_read_response(zw800_dev)) {
        uint16_t cnt = (zw800_dev->buff[10] << 8 | zw800_dev->buff[11]);
        return cnt;
    }

    return -1;
}

int restore_to_default(zw800_t *zw800_dev) {
    zw800_write_cmd(zw800_dev, ZW800_CMD_RESTORE_TO_DEFAULT);
    uint8_t response = zw800_read_response(zw800_dev);
    if (0x00 == response) {
        ESP_LOGI(TAG, "success restore");
        return 0;
    } else {
        ESP_LOGW(TAG, "restore failed with response %d", response);
    }

    return -1;
}

int clear_finger(zw800_t *zw800_dev) {
    zw800_write_cmd(zw800_dev, ZW800_CMD_PS_EMPTY);
    uint8_t response = zw800_read_response(zw800_dev);
    if (0x00 == response) {
        ESP_LOGI(TAG, "success clear all finger data");
        return 0;
    } else {
        ESP_LOGW(TAG, "clear all finger data failed with response %d", response);
    }

    return -1;
}

int identify(zw800_t *zw800_dev) {
    int response;
    zw800_write_cmd(zw800_dev, ZW800_CMD_GET_IMAGE);
    response = zw800_read_response(zw800_dev);
    if (response == 0x00) {
        ESP_LOGI(TAG, "identify get image success");
        zw800_write_cmd(zw800_dev, ZW800_CMD_PS_GetChar);
        response = zw800_read_response(zw800_dev);
        if (response == 0x00) {
            ESP_LOGI(TAG, "identify get char success");
            zw800_write_cmd(zw800_dev, ZW800_CMD_PS_SearchM);
            response = zw800_read_response(zw800_dev);
            uint16_t response_len = zw800_dev->buff[7] << 8 | zw800_dev->buff[8];
            if (response == 0x00 && response_len == 7) {
                int page_id = zw800_dev->buff[10] << 8 | zw800_dev->buff[11];
                int match_score = zw800_dev->buff[12] << 8 | zw800_dev->buff[13];

                esp_event_post(SMART_LOCK_EVENT,
                               FINGER_REC_OK,
                               (void *) page_id,
                               sizeof(page_id),
                               100 / portTICK_PERIOD_MS);

                ESP_LOGI(TAG, "identify success with page_id %d, score %d", page_id, match_score);
                control_led(zw800_dev, 0, 1, 0, 1);
                return page_id;
            } else {
                ESP_LOGW(TAG, "identify failed %d", response);
            }
        } else {
            ESP_LOGW(TAG, "identify get char failed %d", response);
        }
    } else {
        ESP_LOGW(TAG, "identify get image failed %d", response);
    }

    esp_event_post(SMART_LOCK_EVENT,
                   FINGER_REC_OK,
                   NULL,
                   0,
                   100 / portTICK_PERIOD_MS);
    control_led(zw800_dev, 1, 0, 0, 1);
    return 0xff;
}

int auto_identify(zw800_t *zw800_dev) {
    // 1) bit0：采图背光灯控制位，0-LED 长亮，1-LED 获取图像成功后灭
    // 2) bit1：采图预处理控制位，0-关闭预处理，1-打开预处理
    // 3) bit2：注册过程中，是否要求模组在关键步骤，返回当前状态，0-要求返回，1-不要求返回；
    // 4) bit3~bit15：预留。
    ZW800_CMD_PS_AutoIdentify.data[4] = 0x00;
    ZW800_CMD_PS_AutoIdentify.data[5] = 0x00 | (1 << 2);
    zw800_write_cmd(zw800_dev, ZW800_CMD_PS_AutoIdentify);


    while (1) { // TODO timepout
        int response = zw800_read_response(zw800_dev);
        uint16_t response_len = zw800_dev->buff[7] << 8 | zw800_dev->buff[8];
        if (response != 0xff && response_len != 0x08) {
            // failed
            ESP_LOGW(TAG, "auto identify failed, data len not exist response code: %x", response);
            return 0xFF;
        }

        // stage 00 01 05
        uint8_t param = zw800_dev->buff[10];
        uint16_t id = zw800_dev->buff[11] << 8 | zw800_dev->buff[12];
        uint16_t score = zw800_dev->buff[13] << 8 | zw800_dev->buff[14];

        if (response == 0x00 && param == 0x05) {
            // success enter
            ESP_LOGI(TAG, "auto identify success, response: %x, param: %x, id: %x, score: %x", response, param, id,
                     score);
            return response;
        } else if (response == 0x09 && param == 0x05) {
            // no finger find
            ESP_LOGW(TAG, "auto identify no finger found, response: %x, param: %x", response, param);
            return response;
        } else if (response == 0x00) {
            ESP_LOGI(TAG, "auto identify, response: %x, param: %x, id: %x, score: %x", response, param, id, score);
        } else {
            ESP_LOGE(TAG, "auto identify failed, response: %x, param: %x, id: %x, score: %x", response, param, id,
                     score);
            return response;
        }
    }
}

int auto_enroll(zw800_t *zw800_dev, uint16_t page_id) {
    ZW800_CMD_PS_AutoEnroll.data[1] = page_id >> 8;
    ZW800_CMD_PS_AutoEnroll.data[2] = page_id & 0xff;

    // 录入次数
    //ZW800_CMD_PS_AutoEnroll.data[3] = 0x05;

    //1) bit0：采图背光灯控制位，0-LED 长亮，1-LED 获取图像成功后灭
    //2) bit1：采图预处理控制位，0-关闭预处理，1-打开预处理
    //3) bit2：注册过程中，是否要求模组在关键步骤，返回当前状态，0-要求返回，1-不要求返回；
    //4) bit3：是否允许覆盖 ID 号，0-不允许，1-允许；
    //5) bit4：允许指纹重复注册控制位，0-允许，1-不允许
    //6) bit5：注册时，多次指纹采集过程中，是否要求手指离开才能进入下一次指纹图像采集， 0-要求离开；1-不要求离开；
    //7) bit6~bit15：预留。
    ZW800_CMD_PS_AutoEnroll.data[4] = 0x00;
    ZW800_CMD_PS_AutoEnroll.data[5] = 0x00 | (0b00000100);
    zw800_write_cmd(zw800_dev, ZW800_CMD_PS_AutoEnroll);

    while (1) {
        int response = zw800_read_response(zw800_dev);
        uint16_t response_len = zw800_dev->buff[7] << 8 | zw800_dev->buff[8];
        if (response != 0xff && response_len != 5) {
            // failed
            ESP_LOGW(TAG, "auto enroll failed, data len not exist response code: %x", response);
            return 0xFF;
        }

        // get 参数 1 & 2
        uint8_t stage = zw800_dev->buff[10];
        uint8_t param2 = zw800_dev->buff[11];

        if (response != 0x00 && response != 0xFF) {
            ESP_LOGW(TAG, "auto enroll failed, response: %x, param1: %x, param2: %x", response, stage, param2);
            return response;
        }

        if (stage == 0x00) {
            // success enter
            ESP_LOGI(TAG, "success enter auto enroll mode, pageId:%d", page_id);
            vTaskDelay(pdMS_TO_TICKS(5));
        } else if (stage == 0x01) {
            ESP_LOGI(TAG, "auto enroll get image, times: %d", param2);
        } else if (stage == 0x02) {
            ESP_LOGI(TAG, "auto enroll gen char, times: %d", param2);
        } else if (stage == 0x03) {
            ESP_LOGI(TAG, "auto enroll finger leave, times: %d", param2);
        } else if (stage == 0x04) {
            ESP_LOGI(TAG, "auto enroll combine");
        } else if (stage == 0x05 && param2 == 0xf1) {
            ESP_LOGI(TAG, "auto enroll reg verify if exist");
        } else if (stage == 0x06 && param2 == 0xf2) {
            // 模板存储结果
            ESP_LOGI(TAG, "auto enroll save result success, saved to page_id %d, exit auto mode", page_id);
            return response;
        } else {
            ESP_LOGW(TAG, "auto enroll unknown stage : %d", stage);
            return 0xFF;
        }
    }
}

int control_led(zw800_t *zw800_dev, bool r, bool g, bool b, uint8_t repeat) {
    int response;
    //1-普通呼吸灯，2-闪烁灯，3-常开灯，4-常闭灯，5-渐开灯，6-渐闭灯
    ZW800_CMD_PS_CONTROL_LED.data[1] = 0x01; // mode
    //bit0 是蓝灯控制位；bit1 是绿灯控制位；bit2是红灯控制位
    uint8_t start_color = 0x00 | (b ? 1 : 0) | (g ? 1 << 1 : 0) | (r ? 1 << 2 : 0);
    uint8_t end_color = 0x00 | (b ? 1 : 0) | (g ? 1 << 1 : 0) | (r ? 1 << 2 : 0);

    ZW800_CMD_PS_CONTROL_LED.data[2] = start_color; // start color
    ZW800_CMD_PS_CONTROL_LED.data[3] = end_color; // end color

    // 0 无限循环
    ZW800_CMD_PS_CONTROL_LED.data[4] = repeat; // repeat

    zw800_write_cmd(zw800_dev, ZW800_CMD_PS_CONTROL_LED);
    response = zw800_read_response(zw800_dev);
    if (response != 0x00) {
        ESP_LOGW(TAG, "change led color failed");
    }
    return response;
}

// 0 OK
int check_status_ok(zw800_t *zw800_dev) {
    zw800_write_cmd(zw800_dev, ZW800_CMD_ECHO);
    uint8_t response = zw800_read_response(zw800_dev);

    int retry_time = 0;
    while (response != 0x00 && retry_time < 7) {
        ESP_LOGW(TAG, "hand shake with zw800 failed, response code: %x", response);
        vTaskDelay(pdMS_TO_TICKS(50));
        retry_time++;
    }
    return response;
}

static void zw800_task_entry(void *arg) {
    zw800_t *zw800_dev = (zw800_t *) arg;
    event_queue = xQueueCreate(10, sizeof(gpio_num_t));
    while (1) {
        ESP_LOGI(TAG, "CURRENT TOUCH PIN %d", gpio_get_level(zw800_dev->touch_pin));
        while (1) {
            // touch pin is low
            if (!gpio_get_level(zw800_dev->touch_pin)) {
                ESP_LOGI(TAG, "in sleep mode touch pin is low");

                gpio_num_t clicked_gpio;
                if (xQueueReceive(event_queue, &clicked_gpio, portMAX_DELAY)) {
                    gpio_intr_disable(zw800_dev->touch_pin);
                } else {
                    // timeout
                }
            } else {
                break;
            }
        }

        int response = check_status_ok(zw800_dev);
        if (0x00 != response) {
            ESP_LOGE(TAG, "hand shake with zw800 failed");
            //vTaskDelete(NULL);
            //return;
        }

        ESP_LOGI(TAG, "enter for identify mode");
        identify(zw800_dev);
        //auto_identify(zw800_dev);

        do {
            zw800_write_cmd(zw800_dev, ZW800_CMD_SLEEP);
            response = zw800_read_response(zw800_dev);
            if (response != 0x00) {
                ESP_LOGE(TAG, "enter sleep mode failed %X", response);
            } else {
                ESP_LOGI(TAG, "zw800 enter sleep mode");
            }
            //ESP_LOGI(TAG, "TOUCH PIN %d", gpio_get_level(zw800_dev->touch_pin));
            vTaskDelay(pdMS_TO_TICKS(200));
        } while (gpio_get_level(zw800_dev->touch_pin)); // wait finger remove
        gpio_intr_enable(zw800_dev->touch_pin);
    }

    gpio_isr_handler_remove(zw800_dev->touch_pin);
    vTaskDelete(NULL);
}

esp_err_t zw800_init(const zw800_config_t *config) {
    /* Install UART friver */
    uart_config_t uart_config = {
            .baud_rate = config->uart.baud_rate,
            .data_bits = config->uart.data_bits,
            .parity = config->uart.parity,
            .stop_bits = config->uart.stop_bits,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
    };

    zw800.uart_port = config->uart.uart_port;

    // config touch detect isr gpio pin
    zw800.touch_pin = config->touch_pin;
    // setup gpio
    ESP_LOGI(TAG, "touch pin is %d", config->touch_pin);
    gpio_config_t io_config = {
            .pin_bit_mask = (1ull << config->touch_pin),
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_POSEDGE, //interrupt of rising edge GPIO_INTR_HIGH_LEVEL
            .pull_up_en = 0,
            .pull_down_en = 1,
    };
    ESP_ERROR_CHECK(gpio_config(&io_config));

    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(config->touch_pin, gpio_isr_handler, (void *) config->touch_pin);

    if (uart_driver_install(zw800.uart_port, 1024, 0,
                            0, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "install uart driver failed");
        goto err_uart_install;
    }
    if (uart_param_config(zw800.uart_port, &uart_config) != ESP_OK) {
        ESP_LOGE(TAG, "config uart parameter failed");
        goto err_uart_config;
    }
    if (uart_set_pin(zw800.uart_port, config->uart.tx_pin, config->uart.rx_pin,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "config uart gpio failed");
        goto err_uart_config;
    }
    zw800.buff = calloc(1, BUF_SIZE);
    if (!zw800.buff) {
        ESP_LOGE(TAG, "calloc memory for runtime buffer failed");
        goto err_buffer;
    }

    /* Create NMEA Parser task */
    BaseType_t err = xTaskCreate(
            zw800_task_entry,
            "zw800_task",
            3072,
            &zw800,
            uxTaskPriorityGet(NULL),
            &zw800.tsk_hdl);
    if (err != pdTRUE) {
        ESP_LOGE(TAG, "create zw800 task failed");
        goto err_task_create;
    }
    ESP_LOGI(TAG, "zw800 task init OK");
    return ESP_OK;
    /*Error Handling*/
    err_task_create:
    err_uart_install:
    uart_driver_delete(zw800.uart_port);
    err_uart_config:
    err_buffer:
    free(zw800.buff);
    return ESP_FAIL;
}

esp_err_t zw800_clear_all_finger() {
    int response = clear_finger(&zw800);
    restore_to_default(&zw800);

    return response == 0x00 ? ESP_OK : ESP_FAIL;
}

esp_err_t zw800_add_finger(uint8_t id) {
    int response = check_status_ok(&zw800);
    if (0x00 != response) {
        ESP_LOGE(TAG, "add finger hand shake failed");
        return ESP_FAIL;
    }

    gpio_intr_disable(zw800.touch_pin);

    // check template num
    if (id == 0) {
        int temp_num = get_valid_template_num(&zw800);
        ESP_LOGI(TAG, "zw800 template num: %d", temp_num);
        if (temp_num >= 0) {
            id = (temp_num + 1);
        } else {
            ESP_LOGE(TAG, "zw800 read template num error");
            return ESP_FAIL;
        }
    }

    // enter auto enroll mode
    ESP_LOGI(TAG, "enter auto enroll mode for id %d", id);
    auto_enroll(&zw800, id);

    zw800_write_cmd(&zw800, ZW800_CMD_SLEEP);
    response = zw800_read_response(&zw800);

    gpio_intr_enable(zw800.touch_pin);

    return ESP_OK;
}
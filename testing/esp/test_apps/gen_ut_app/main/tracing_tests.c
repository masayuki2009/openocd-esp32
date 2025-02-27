#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/timer.h"
#include "sdkconfig.h"
#include "gen_ut_app.h"
#include "esp_app_trace.h"

#define LOG_LOCAL_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include "esp_log.h"
const static char *TAG = "tracing_tests";

// need to protect part of the file with macro to get compiled successfully for the oldest supported IDF ver,
// this comparison should be updated if oldest supported IDF ver is increased enough to provide those features
#if UT_IDF_VER >= MAKE_UT_IDF_VER(4,0,0,0)
#if CONFIG_HEAP_TRACING
#include "esp_heap_trace.h"
#endif //CONFIG_HEAP_TRACING
#if CONFIG_SYSVIEW_ENABLE
#include "esp_sysview_trace.h"
#endif

struct trace_test_task_arg;
typedef void (*do_trace_test_t)(struct trace_test_task_arg* task);
typedef int (*trace_printf_t)(const char *fmt, ...);
typedef int (*trace_flush_t)(uint32_t tmo);

typedef struct trace_test_task_arg {
    do_trace_test_t test_func;
    TaskHandle_t    other_task;
    trace_printf_t  trace_printf;
    trace_flush_t   trace_flush;
} trace_test_task_arg_t;

#if CONFIG_SYSVIEW_ENABLE
int do_trace_printf(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    int ret = esp_sysview_vprintf(fmt, ap);
    va_end(ap);

    return ret;
}

static int do_trace_flush(uint32_t tmo)
{
    return esp_sysview_flush(tmo);
}
#else
int do_trace_printf(const char *fmt, ...)
{
    return 0;
}

static int do_trace_flush(uint32_t tmo)
{
    return ESP_OK;
}
#endif

#if CONFIG_HEAP_TRACING
static void do_trace_test_heap_log(uint32_t num, trace_printf_t trace_printf)
{
    volatile TaskHandle_t curr_task = xTaskGetCurrentTaskHandle();

    void *a = malloc(64); TEST_BREAK_LOC(malloc0);
    trace_printf("task[%p]: Sample print 0x%lx 0x%hx 0x%x %lu %hu %u %ld %hd %d %c\n",
        curr_task, num, num, num, num, num, num, num, num, num, 'A' + (num  % ('Z' - 'A')));
    num++;
    void *b = malloc(96); TEST_BREAK_LOC(malloc1);
    trace_printf("task[%p]: Sample print 0x%lx 0x%hx 0x%x %lu %hu %u %ld %hd %d %c\n",
        curr_task, num, num, num, num, num, num, num, num, num, 'A' + (num  % ('Z' - 'A')));
    num++;
    free(a); TEST_BREAK_LOC(free0);
    trace_printf("task[%p]: Sample print 0x%lx 0x%hx 0x%x %lu %hu %u %ld %hd %d %c\n",
        curr_task, num, num, num, num, num, num, num, num, num, 'A' + (num  % ('Z' - 'A')));
    num++;
    b = malloc(10); TEST_BREAK_LOC(malloc2);
    trace_printf("task[%p]: Sample print 0x%lx 0x%hx 0x%x %lu %hu %u %ld %hd %d %c\n",
        curr_task, num, num, num, num, num, num, num, num, num, 'A' + (num  % ('Z' - 'A')));
    num++;
    b = malloc(23); TEST_BREAK_LOC(malloc3);
    trace_printf("task[%p]: Sample print 0x%lx 0x%hx 0x%x %lu %hu %u %ld %hd %d %c\n",
        curr_task, num, num, num, num, num, num, num, num, num, 'A' + (num  % ('Z' - 'A')));
    num++;
    free(b); TEST_BREAK_LOC(free1);
    trace_printf("task[%p]: Sample print 0x%lx 0x%hx 0x%x %lu %hu %u %ld %hd %d %c\n",
        curr_task, num, num, num, num, num, num, num, num, num, 'A' + (num  % ('Z' - 'A')));
    __asm__ volatile (
        ".global _do_trace_test_heap_log_end\n" \
        ".type   _do_trace_test_heap_log_end,@function\n" \
        "_do_trace_test_heap_log_end:\n" \
        "   nop\n" \
        :::);
}

static void trace_test_heap_log_main(trace_test_task_arg_t *arg)
{
    static uint32_t num = 0;

    if(heap_trace_init_tohost() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init heap trace!");
        return;
    }
    heap_trace_start(HEAP_TRACE_LEAKS);
#if !CONFIG_FREERTOS_UNICORE
    xTaskNotify(arg->other_task, 0, eNoAction);
#endif

    do_trace_test_heap_log(num, arg->trace_printf); TEST_BREAK_LOC(trace_test_func_call);
    num += 10;

#if !CONFIG_FREERTOS_UNICORE
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
#endif
    // does trace data flush implicitly
    heap_trace_stop();
}

#if !CONFIG_FREERTOS_UNICORE
static void trace_test_heap_log_slave(trace_test_task_arg_t *arg)
{
    static uint32_t num = 0;
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
    do_trace_test_heap_log(num, arg->trace_printf); TEST_BREAK_LOC(trace_test_func_call);
    num += 10;
    xTaskNotify(arg->other_task, 0, eNoAction);
}
#endif
#endif //CONFIG_HEAP_TRACING

static __attribute__((noinline)) void _trace_test_log_continuous_start(void)
{
    __asm__ volatile (
        "   nop\n" \
        :::);
}

static __attribute__((noinline)) void _trace_test_log_continuous_stop(void)
{
    __asm__ volatile (
        "   nop\n" \
        :::);
}

static __attribute__((noinline)) void do_trace_test_log_continuous(uint32_t num, trace_printf_t trace_printf)
{
    volatile TaskHandle_t curr_task = xTaskGetCurrentTaskHandle();

    for (int i = 0; i < 400; i++) {
        trace_printf("task[%p]: Sample print 0x%lx 0x%hx 0x%x %lu %hu %u %ld %hd %d %c\n",
            curr_task, num, num, num, num, num, num, num, num, num, 'A' + (num  % ('Z' - 'A')));
        num++;
        vTaskDelay(1);
    }
    __asm__ volatile (
        ".global _trace_test_log_continuous_end\n" \
        ".type   _trace_test_log_continuous_end,@function\n" \
        "_trace_test_log_continuous_end:\n" \
        "   nop\n" \
        :::);
}

static __attribute__((noinline)) void trace_test_log_continuous_main(trace_test_task_arg_t *arg)
{
    static uint32_t num = 0;

    _trace_test_log_continuous_start();
#if !CONFIG_FREERTOS_UNICORE
    xTaskNotify(arg->other_task, 0, eNoAction);
#endif

    do_trace_test_log_continuous(num, arg->trace_printf);
    num += 10;

#if !CONFIG_FREERTOS_UNICORE
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
#endif
    arg->trace_flush(ESP_APPTRACE_TMO_INFINITE);
    _trace_test_log_continuous_stop();
}

#if !CONFIG_FREERTOS_UNICORE
static void trace_test_log_continuous_slave(trace_test_task_arg_t *arg)
{
    static uint32_t num = 0;
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
    do_trace_test_log_continuous(num, arg->trace_printf);
    num += 10;
    xTaskNotify(arg->other_task, 0, eNoAction);
}
#endif

static void trace_test_task(void *pvParameter)
{
    trace_test_task_arg_t *arg = (trace_test_task_arg_t *)pvParameter;
#if !CONFIG_FREERTOS_UNICORE
    // wait until other task handle is known
    while(arg->other_task == NULL) {
        vTaskDelay(1);
    }
#endif
    arg->test_func(arg);
    while(1) {
        vTaskDelay(1);
    }
}
#endif // UT_IDF_VER

struct os_trace_task_arg {
    int tim_grp;
    int tim_id;
    uint32_t tim_period;
    uint32_t task_period;
};

static void os_trace_test_timer_isr(void *arg)
{
    struct os_trace_task_arg *tim_arg = (struct os_trace_task_arg *)arg;

    // ESP_LOGI(TAG, "Failed to start timer (%d)!", res);
    test_timer_rearm(tim_arg->tim_grp, tim_arg->tim_id);
}

static void os_trace_test_task(void *pvParameter)
{
    int i = 0;
    struct os_trace_task_arg *arg = (struct os_trace_task_arg *)pvParameter;

    test_timer_init(arg->tim_grp, arg->tim_id, arg->tim_period);
    int res = timer_isr_register(arg->tim_grp, arg->tim_id, os_trace_test_timer_isr, arg, 0, NULL);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register timer ISR (%d)!", res);
        return;
    }
    res = timer_start(arg->tim_grp, arg->tim_id);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer (%d)!", res);
        return;
    }

    while(1) {
        printf("task[%p] work %d\n", xTaskGetCurrentTaskHandle(), i++);
        vTaskDelay(arg->task_period / portTICK_PERIOD_MS);
    }
}

/* CONFIG_ESP32_APPTRACE_ENABLE is for IDF <= 4.0 */
#if defined(CONFIG_APPTRACE_ENABLE) || defined(CONFIG_ESP32_APPTRACE_ENABLE)

static int apptrace_writefn(void* cookie, const char* data, int size)
{
    int res = esp_apptrace_write(ESP_APPTRACE_DEST_TRAX, data, size, 1000);
    if (res != ESP_OK) {
        return 0;
    }
    /* this function may fail if host is busy and is not able to read data (flushed previously) within 1 ms  */
    esp_apptrace_flush(ESP_APPTRACE_DEST_TRAX, 1000);
    return size;
}

static void raw_trace_log_done(void)
{
    __asm__ __volatile__("nop");
}

static void raw_trace_log(void* arg)
{
    uint32_t iter_count = (uint32_t)arg;
    stdout = fwopen(NULL, &apptrace_writefn);
    static char stdout_buf[128];
    setvbuf(stdout, stdout_buf, _IOLBF, sizeof(stdout_buf));

    for (int i = 0; i < iter_count; ++i) {
        printf("[%d %*.s]\n", i, i * 20, "test");
    }
    /* ensure that all data are gone to the host in case the last call to esp_apptrace_flush() from apptrace_writefn() failed */
    esp_apptrace_flush(ESP_APPTRACE_DEST_TRAX, ESP_APPTRACE_TMO_INFINITE);
    raw_trace_log_done();
    vTaskDelete(NULL);
}

static void raw_trace_log_periodic(void* arg)
{
    uint32_t delay = (uint32_t)arg;
    stdout = fwopen(NULL, &apptrace_writefn);
    static char stdout_buf[128];
    setvbuf(stdout, stdout_buf, _IOLBF, sizeof(stdout_buf));

    while (!esp_apptrace_host_is_connected(ESP_APPTRACE_DEST_TRAX))
        vTaskDelay(1);  

    int cnt = 0;
    while (1) {
        printf("apptrace test line#%d\n", cnt++);
        vTaskDelay(delay / portTICK_PERIOD_MS);  
    }
    
}
#endif // CONFIG_APPTRACE_ENABLE

ut_result_t tracing_test_do(int test_num)
{
#if UT_IDF_VER >= MAKE_UT_IDF_VER(4,0,0,0)
    static trace_test_task_arg_t task_args[2];
    memset(task_args, 0, sizeof(task_args));
#endif // UT_IDF_VER

    switch(test_num) {
#if UT_IDF_VER >= MAKE_UT_IDF_VER(4,0,0,0)
#if CONFIG_HEAP_TRACING
        case 500:
        {
            task_args[0].test_func = (do_trace_test_t)trace_test_heap_log_main;
            task_args[0].trace_printf = do_trace_printf;
            task_args[0].trace_flush = do_trace_flush;
            xTaskCreatePinnedToCore(trace_test_task, "trace_task0", 2048, (void *)&task_args[0], 5, &task_args[1].other_task, 0);
#if !CONFIG_FREERTOS_UNICORE
            task_args[1].test_func = (do_trace_test_t)trace_test_heap_log_slave;
            task_args[1].trace_printf = do_trace_printf;
            task_args[1].trace_flush = do_trace_flush;
            xTaskCreatePinnedToCore(trace_test_task, "trace_task1", 2048, (void *)&task_args[1], 5, &task_args[0].other_task, 1);
#endif
            break;
        }
#endif //CONFIG_HEAP_TRACING
        case 501:
        {
            task_args[0].test_func = (do_trace_test_t)trace_test_log_continuous_main;
            task_args[0].trace_printf = do_trace_printf;
            task_args[0].trace_flush = do_trace_flush;
            xTaskCreatePinnedToCore(trace_test_task, "trace_task0", 2048, (void *)&task_args[0], 5, &task_args[1].other_task, 0);
#if !CONFIG_FREERTOS_UNICORE
            task_args[1].test_func = (do_trace_test_t)trace_test_log_continuous_slave;
            task_args[1].trace_printf = do_trace_printf;
            task_args[1].trace_flush = do_trace_flush;
            xTaskCreatePinnedToCore(trace_test_task, "trace_task1", 2048, (void *)&task_args[1], 5, &task_args[0].other_task, 1);
#endif
            break;
        }
#endif // UT_IDF_VER
        case 502:
        {
            static struct os_trace_task_arg task_args[2] = {
                { .tim_grp = TIMER_GROUP_1, .tim_id = TIMER_0, .tim_period = 300000UL /*us*/, .task_period = 500 /*ms*/},
                { .tim_grp = TIMER_GROUP_1, .tim_id = TIMER_1, .tim_period = 500000UL /*us*/, .task_period = 2000 /*ms*/}
            };
            xTaskCreatePinnedToCore(os_trace_test_task, "trace_task0", 2048, (void *)&task_args[0], 5, NULL, 0);
#if !CONFIG_FREERTOS_UNICORE
            xTaskCreatePinnedToCore(os_trace_test_task, "trace_task1", 2048, (void *)&task_args[1], 5, NULL, 1);
#endif
            break;
        }
#if defined(CONFIG_APPTRACE_ENABLE) || defined(CONFIG_ESP32_APPTRACE_ENABLE)
        case 503:
        {
            xTaskCreate(raw_trace_log, "raw_trace_log", 2048, (void *)10, 5, NULL);
            break;
        }
        case 504:
        {
            xTaskCreate(raw_trace_log, "raw_trace_log", 2048, (void *)100, 5, NULL);
            break;
        }
        case 505:
        {
            xTaskCreate(raw_trace_log_periodic, "raw_trace_log_periodic", 2048, (void *)100, 5, NULL);
            break;
        }
#endif //CONFIG_APPTRACE_ENABLE
        default:
            return UT_UNSUPPORTED;
    }
    return UT_OK;
}

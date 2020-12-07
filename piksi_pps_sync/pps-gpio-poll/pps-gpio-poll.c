#define pr_fmt(fmt)                                                            \
  "pps-gpio-poll"                                                              \
  ": " fmt

#include <linux/gpio.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/pps_kernel.h>
#include <linux/workqueue.h>

/* Struct containing static info. */
struct pps_gpio_data {
  int iter;               /* Maximum number of GPIO reads in wait loop */
  struct pps_device *pps; /* The PPS device */
  struct pps_source_info pps_info;    /* PPS source information */
  struct workqueue_struct *workqueue; /* Workqueue */
};

struct pps_gpio_data data;

static void gpio_poll(struct work_struct *work);
static void gpio_wait(struct work_struct *work);

struct delayed_work wait;
DECLARE_DELAYED_WORK(wait, &gpio_wait);

struct work_struct poll;
DECLARE_WORK(poll, &gpio_poll);

static int gpio = 0;
module_param(gpio, int, S_IRUSR);
MODULE_PARM_DESC(gpio, "PPS GPIO");

static int pulse_width = 2;
module_param(pulse_width, int, S_IRUSR);
MODULE_PARM_DESC(pulse_width, "PPS pulse width [ms]");

static int pps_gpio_add(void) {
  int i, ret;
  ktime_t ts1, ts2;
  int pps_default_params = PPS_CAPTUREASSERT | PPS_OFFSETASSERT;
  u64 dur = 0;
  u64 min = 0xFFFFFFFFFFFFFFFF;
  u64 max = 0;
  u64 avg = 0;

  /* GPIO setup */
  ret = gpio_request(gpio, "PPS");
  if (ret) {
    pr_warning("Failed to request PPS GPIO %u\n", gpio);
    return -EINVAL;
  }
  ret = gpio_direction_input(gpio);
  if (ret) {
    pr_warning("Failed to set PPS GPIO direction\n");
    gpio_free(gpio);
    return -EINVAL;
  }

  /* register PPS source */
  snprintf(data.pps_info.name, PPS_MAX_NAME_LEN - 1, "pps-gpio-poll.0");
  data.pps_info.mode = PPS_CAPTUREASSERT | PPS_OFFSETASSERT | PPS_OFFSETCLEAR |
                       PPS_CANWAIT | PPS_TSFMT_TSPEC;
  data.pps_info.owner = THIS_MODULE;
  data.pps_info.dev = NULL;
  data.pps = pps_register_source(&data.pps_info, pps_default_params);
  if (data.pps == NULL) {
    pr_err("Failed to register GPIO PPS source\n");
    gpio_free(gpio);
    return -EINVAL;
  }

  /* Get GPIO timing precision */
  for (i = 0; i < 1000; i++) {
    ts1 = ktime_get();
    gpio_get_value_cansleep(gpio);
    ts2 = ktime_get();
    dur = ktime_to_ns(ts2) - ktime_to_ns(ts1);
    min = dur < min ? dur : min;
    max = dur > max ? dur : max;
    avg += dur / 1000;
  }

  pr_info("Registered GPIO %d as PPS source. Precision [ns] avg: %llu min: "
          "%llu, max: %llu\n",
          gpio, avg, min, max);

  /* Maximum number of GPIO reads. */
  data.iter = 4 * pulse_width * 1000000 / min;
  pr_info("Maximum number of GPIO polls: %d", data.iter);

  return 0;
}

static void gpio_poll(struct work_struct *work) {
  int ret;

  if (gpio_get_value_cansleep(gpio)) {
    /* got a PPS event, start busy waiting shortly before new event. */
    ret = queue_delayed_work(data.workqueue, &wait,
                             msecs_to_jiffies(1000 - 2 * pulse_width));
    if (!ret) {
      pr_err("Cannot queue PPS waiting work.\n");
    } else {
      pr_info("Received initial PPS. Busy waiting for next.\n");
    }
  } else {
    /* Probe GPIO again. */
    ret = queue_work(data.workqueue, &poll);
    if (!ret) {
      pr_err("Cannot queue PPS probing work.\n");
    }
  }

  return;
}

static void gpio_wait(struct work_struct *work) {
  int i, ret;
  bool has_pps = false;
  unsigned long flags;
  struct pps_event_time ts;
  int value = gpio_get_value_cansleep(gpio);

  if (value) {
    pr_warn("PPS already high. Switch back into probing PPS.\n");
    ret = queue_work(data.workqueue, &poll);
    if (!ret) {
      pr_err("Cannot return into PPS probing.\n");
    }
    return;
  }

  /* read the GPIO value until the PPS event occurs */
  local_irq_save(flags);
  for (i = 0; likely(i < data.iter && !value); i++) {
    pps_get_ts(&ts);
    value = gpio_get_value_cansleep(gpio);
  }
  local_irq_restore(flags);

  if (i < data.iter) {
    /* Caught PPS. */
    has_pps = true;
    ret = queue_delayed_work(data.workqueue, &wait,
                             msecs_to_jiffies(1000 - 2 * pulse_width));
    if (!ret) {
      pr_err("Cannot queue next PPS waiting work.\n");
    }
  } else {
    pr_warn("Missed PPS pulse. Switch into probing PPS.\n");
    ret = queue_work(data.workqueue, &poll);
    if (!ret) {
      pr_err("Cannot return into PPS probing.\n");
    }
  }

  if (has_pps) {
    pps_event(data.pps, &ts, PPS_CAPTUREASSERT, NULL);
  }

  return;
}

static int __init pps_gpio_init(void) {
  int ret;

  ret = pps_gpio_add();
  if (ret < 0)
    return ret;

  /* Setup workqueue */
  data.workqueue = create_singlethread_workqueue("pps_polling");

  // Probe GPIO.
  ret = queue_work(data.workqueue, &poll);
  if (!ret) {
    return -EINVAL;
  }

  return 0;
}

static int pps_gpio_remove(void) {
  pps_unregister_source(data.pps);
  gpio_free(gpio);
  pr_info("Removed GPIO %d as PPS source\n", gpio);
  return 0;
}

static void __exit pps_gpio_exit(void) {
  cancel_work_sync(&poll);
  cancel_delayed_work_sync(&wait);
  flush_workqueue(data.workqueue);
  destroy_workqueue(data.workqueue);
  pps_gpio_remove();
}

module_init(pps_gpio_init);
module_exit(pps_gpio_exit);

MODULE_AUTHOR("Rik Baehnemann");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.0.1");

#define pr_fmt(fmt)                                                            \
  "pps-gpio-modprobe"                                                          \
  ": " fmt

#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/pps_kernel.h>

/* Struct containing static info. */
struct pps_gpio_data {
  int gpio;                    /* The GPIO pin */
  int irq;                     /* The IRQ used as PPS source */
  struct pps_device *pps;      /* The PPS device */
  struct pps_source_info info; /* PPS source information */
};

struct pps_gpio_data in_data;

static int gpio = -1;
module_param(gpio, int, S_IRUSR);

/*
 * Report the PPS event
 */
static irqreturn_t pps_gpio_irq_handler(int irq, void *data) {
  const struct pps_gpio_data *info;
  struct pps_event_time ts;
  int rising_edge;

  /* Get the time stamp first */
  pps_get_ts(&ts);

  info = data;

  rising_edge = gpio_get_value(info->gpio);
  if (rising_edge)
    pps_event(info->pps, &ts, PPS_CAPTUREASSERT, NULL);

  return IRQ_HANDLED;
}

static int pps_gpio_add(void) {
  int pps_default_params;
  int ret;
  int irq_flags;

  /* Copy user params. */
  in_data.gpio = gpio;

  /* GPIO setup */
  ret = gpio_request(in_data.gpio, "PPS");
  if (ret) {
    pr_err("Failed to request PPS GPIO %u\n", in_data.gpio);
    return -EINVAL;
  }
  ret = gpio_direction_input(in_data.gpio);
  if (ret) {
    pr_err("Failed to set PPS GPIO direction: %d\n", ret);
    gpio_free(in_data.gpio);
    return -EINVAL;
  }

  /* IRQ setup */
  ret = gpio_to_irq(in_data.gpio);
  if (ret < 0) {
    pr_err("Failed to map GPIO to IRQ: %d\n", ret);
    gpio_free(in_data.gpio);
    return -EINVAL;
  }
  in_data.irq = ret;

  /* PPS setup */
  snprintf(in_data.info.name, PPS_MAX_NAME_LEN - 1, "pps-gpio-modprobe.0");
  in_data.info.mode = PPS_CAPTUREASSERT | PPS_OFFSETASSERT | PPS_ECHOASSERT |
                      PPS_CANWAIT | PPS_TSFMT_TSPEC;
  in_data.info.owner = THIS_MODULE;
  in_data.info.dev = NULL;
  pps_default_params = PPS_CAPTUREASSERT | PPS_OFFSETASSERT;
  in_data.pps = pps_register_source(&in_data.info, pps_default_params);
  if (in_data.pps == NULL) {
    pr_err("Failed to register GPIO PPS source\n");
    free_irq(in_data.irq, &in_data);
    gpio_free(in_data.gpio);
    return -EINVAL;
  }

  /* Register IRQ interrupt handler */
  irq_flags = IRQF_TRIGGER_RISING | IRQF_EARLY_RESUME | IRQF_NOBALANCING |
              IRQF_NO_THREAD;
  ret = request_threaded_irq(in_data.irq, pps_gpio_irq_handler, NULL,
                             irq_flags, in_data.info.name, &in_data);
  if (ret) {
    pr_err("Failed to register aquire IRQ %d\n", in_data.irq);
    pps_unregister_source(in_data.pps);
    gpio_free(in_data.gpio);
    return -EINVAL;
  }

  pr_info("Registered GPIO %d as PPS source with IRQ %d\n", in_data.gpio,
          in_data.irq);
  return 0;
}

static int pps_gpio_remove(void) {
  free_irq(in_data.irq, &in_data);
  pps_unregister_source(in_data.pps);
  gpio_free(in_data.gpio);
  pr_info("Removed GPIO %d as PPS source\n", in_data.gpio);
  return 0;
}

static int __init pps_gpio_init(void) {
  int ret = pps_gpio_add();
  if (ret < 0)
    return ret;
  return 0;
}

static void __exit pps_gpio_exit(void) { pps_gpio_remove(); }

module_init(pps_gpio_init);
module_exit(pps_gpio_exit);

MODULE_AUTHOR("Rik Baehnemann");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.0.1");

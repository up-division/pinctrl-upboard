// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel Low Power Subsystem PWM controller PCI driver
 *
 * Copyright (C) 2014, Intel Corporation
 *
 * Derived from the original pwm-lpss.c
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>

#include "pwm-lpss.h"
#include "protos.h"

static int pwm_lpss_probe_pci(struct pci_dev *pdev,
			      const struct pci_device_id *id)
{
	const struct pwm_lpss_boardinfo *info;
	struct pwm_lpss_chip *lpwm;
	struct pwm_chip *chip;
	int err;
	
	dev_info(&pdev->dev,"pwm_lpss_probe_pci");

	err = pcim_enable_device(pdev);
	if (err < 0)
		return err;

	err = pcim_iomap_regions(pdev, BIT(0), pci_name(pdev));
	if (err)
		return err;

	info = (struct pwm_lpss_boardinfo *)id->driver_data;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 9, 0)
	chip = upboard_pwm_lpss_probe(&pdev->dev, pcim_iomap_table(pdev)[0], info);
	if (IS_ERR(chip))
		return PTR_ERR(chip);
#else
	lpwm = upboard_pwm_lpss_probe(&pdev->dev, pcim_iomap_table(pdev)[0], info);
	if (IS_ERR(lpwm))
		return PTR_ERR(lpwm);

	pci_set_drvdata(pdev, lpwm);
#endif
	pm_runtime_put(&pdev->dev);
	pm_runtime_allow(&pdev->dev);

	return 0;
}

static void pwm_lpss_remove_pci(struct pci_dev *pdev)
{
	pm_runtime_forbid(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
}

static int pwm_lpss_runtime_suspend_pci(struct device *dev)
{
	/*
	 * The PCI core will handle transition to D3 automatically. We only
	 * need to provide runtime PM hooks for that to happen.
	 */
	return 0;
}

static int pwm_lpss_runtime_resume_pci(struct device *dev)
{
	return 0;
}

#if TYPES_NO_ERROR_CODE==1
static const struct dev_pm_ops pwm_lpss_pci_pm = {
	SET_RUNTIME_PM_OPS(pwm_lpss_runtime_suspend_pci,
			   pwm_lpss_runtime_resume_pci, NULL)
};
#else
static DEFINE_RUNTIME_DEV_PM_OPS(pwm_lpss_pci_pm,
				 pwm_lpss_runtime_suspend_pci,
				 pwm_lpss_runtime_resume_pci,
				 NULL);
#endif				 

static const struct pci_device_id pwm_lpss_pci_ids[] = {
	{ PCI_VDEVICE(INTEL, 0x0ac8), (unsigned long)&pwm_lpss_bxt_info},
	{ PCI_VDEVICE(INTEL, 0x0f08), (unsigned long)&pwm_lpss_byt_info},
	{ PCI_VDEVICE(INTEL, 0x0f09), (unsigned long)&pwm_lpss_byt_info},
	{ PCI_VDEVICE(INTEL, 0x11a5), (unsigned long)&pwm_lpss_tng_info},
	{ PCI_VDEVICE(INTEL, 0x1ac8), (unsigned long)&pwm_lpss_bxt_info},
	{ PCI_VDEVICE(INTEL, 0x2288), (unsigned long)&pwm_lpss_bsw_info},
	{ PCI_VDEVICE(INTEL, 0x2289), (unsigned long)&pwm_lpss_bsw_info},
	{ PCI_VDEVICE(INTEL, 0x31c8), (unsigned long)&pwm_lpss_bxt_info},
	{ PCI_VDEVICE(INTEL, 0x5ac8), (unsigned long)&pwm_lpss_bxt_info},
	{ },
};
MODULE_DEVICE_TABLE(pci, pwm_lpss_pci_ids);

static struct pci_driver pwm_lpss_driver_pci = {
	.name = "pwm-lpss",
	.id_table = pwm_lpss_pci_ids,
	.probe = pwm_lpss_probe_pci,
	.remove = pwm_lpss_remove_pci,
	.driver = {
#if TYPES_NO_ERROR_CODE==1
		.pm = &pwm_lpss_pci_pm,
#else
		.pm = pm_ptr(&pwm_lpss_pci_pm),
#endif
	},
};
module_pci_driver(pwm_lpss_driver_pci);

MODULE_DESCRIPTION("PWM PCI driver for Intel LPSS");
MODULE_LICENSE("GPL v2");
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 13, 0)
MODULE_IMPORT_NS(PWM_LPSS);
#else
MODULE_IMPORT_NS("PWM_LPSS");
#endif

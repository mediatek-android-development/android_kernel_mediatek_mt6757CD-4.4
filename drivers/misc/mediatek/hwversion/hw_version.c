/* Copyright (C) 1996-2016, TP-LINK TECHNOLOGIES CO., LTD.
 *
 * File name: hwversion.c
 *
 * Description: define a function to get pcb version voltage from cmdline
 *
 * Author: Li Guanxiong Modify
 *
 * Email: liguanxiong@tp-link.com.cn
 */
#include <linux/module.h>
#include <linux/string.h>
#include <linux/printk.h>
#include <linux/init.h>

#include <linux/seq_file.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

extern char* saved_command_line;

#define PCB_VERSION_VOLTAGE_MAX 10

static int pcb_version_voltage = 0;
static int cmdline_parse_finish = 0;

int get_pcb_version_voltage(void)
{
	char *p = NULL;
	char *q = NULL;
	int i = 0;
	int ret = -1;
	int tmp_voltage = 0;
	char voltage[PCB_VERSION_VOLTAGE_MAX] = {0};
	if (cmdline_parse_finish == 1) {
		return pcb_version_voltage;
	}
	pr_debug("%s:saved_command_line is %s\n", __func__, saved_command_line);
	p = strstr(saved_command_line, "hwversion_voltage=");

	if (p == NULL) {
		return 0;
	}
	p += strlen("hwversion_voltage=");

	if ((p - saved_command_line) > strlen(saved_command_line + 1)) {
		  return 0;
	}

	pr_debug("%s, %s\n", __func__, p);
	q = p;
	while (*q != ' ' && *q != '\0' && i < (sizeof(voltage) - 1)) {
		q++;
		i++;
	}
	memset((void *)voltage, 0, sizeof(voltage));
	strncpy((char *)voltage, (const char *)p, (int)(q - p));
	voltage[q - p + 1] = '\0';
	ret = sscanf(voltage, "%d", &tmp_voltage);
	if (ret != 1) {
		return 0;
	} else {
		pcb_version_voltage = tmp_voltage;
		cmdline_parse_finish = 1;
	}
	pr_info("%s:pcb hw version voltage is %d\n", __func__, pcb_version_voltage);
	return pcb_version_voltage;
}
EXPORT_SYMBOL(get_pcb_version_voltage);

/* [liguanxiong start] */
char* pcb_version_s[] = {
	"EVT",
	"DVT1",
	"DVT2",
	"DVT3",
	"PVT",
	"MP",
	"EVT1",
	"EVT2",
	"DVT1/DVT2",
	"PVT/MP",
};
enum pcb_version {
	EVT = 0,
	DVT1,
	DVT2,
	DVT3,
	PVT,
	MP,
	EVT1,
	EVT2,
	DVT1_DVT2,
	PVT_MP,
};

char* get_pcb_version_s(void)
{
	int index = 0;
	int pcb_version_voltage = 0;
	pcb_version_voltage = get_pcb_version_voltage();

	/* [TP908] EVT1:0.9V EVT2:0.6V DVT1/DVT2:1.2V PVT/MP:0V */
	if (pcb_version_voltage > 1050) {
		index = DVT1_DVT2;
	}else if (pcb_version_voltage > 750) {
		index = EVT1;
	}else if (pcb_version_voltage > 450) {
		index = EVT2;
	} else {
		index = PVT_MP;
	}
	return pcb_version_s[index];
}
/* [liguanxiong end] */

static int __init hw_version_init(void)
{
	get_pcb_version_voltage();
	return 0;
}


#define USER_ROOT_DIR "pcb"
#define USER_ENTRY1   "hwversion_voltage"

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
extern int auxadc_test(void);
static int hw_read(struct seq_file *m, void *v)
{
	/* [liguanxiong] */
        int data[4] = {0, 0, 0, 0};
        int rawvalue    = 0;
	int ret = -1;
	int sum = 0;
	ret = IMM_GetOneChannelValue(3, data, &rawvalue);
	sum = data[0]*1000 + data[1]*10;
	printk("__LGX, ret = %d, sum = %d\n", ret, sum);
	printk("__LGX, hw_read!\n");
	seq_printf(m, "%s(%dmV)\n", get_pcb_version_s(), get_pcb_version_voltage());
	return 0;
}

static int hw_version_open(struct inode *inode, struct file *file) {
	return single_open(file, hw_read, NULL);
}
static const struct file_operations hwversion_fops = {
	.owner = THIS_MODULE,
	.open = hw_version_open,
	.read = seq_read,
	.write = NULL,
	.release = single_release,
};

static int hwversion_module_init(void)
{

	struct proc_dir_entry *pt_root = NULL;
	pt_root = proc_mkdir(USER_ROOT_DIR, NULL);
	if (NULL == pt_root) {
		printk(KERN_ALERT "Create dir /proc/%s error!\n",  USER_ROOT_DIR);
		return -1;
	}
	printk(KERN_INFO "Create dir /proc/%s\n", USER_ROOT_DIR);
	if(!proc_create("hwversion_voltage", S_IRUGO | S_IWUSR, pt_root, &hwversion_fops)) {
		goto err_out;
	}
	return 0;
err_out:
	remove_proc_entry(USER_ROOT_DIR, pt_root);
	return -1;
}

static void hwversion_module_exit(void)
{

	printk(KERN_INFO "All Proc Entry Removed!\n");
}

early_initcall(hw_version_init);
module_init(hwversion_module_init);
module_exit(hwversion_module_exit);
MODULE_DESCRIPTION("Mediatek Boot Reason Driver");
MODULE_LICENSE("GPL");

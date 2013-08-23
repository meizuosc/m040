#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <plat/cpu.h>
#include <mach/mx-boot-type.h>

static int uboot_svn_version_proc_show(struct seq_file *m, void *v)
{
	char uboot_svn_version[6];
	sprintf(uboot_svn_version, "%d", bootinfo.uboot_svn_version);
	seq_printf(m, "%s\n", uboot_svn_version);

	return 0;
}

static int uboot_svn_version_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, uboot_svn_version_proc_show, NULL);
}

static const struct file_operations uboot_version_proc_fops = {
	.open		= uboot_svn_version_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_uboot_version_init(void)
{
	proc_create("uboot_svn_version", 0, NULL, &uboot_version_proc_fops);
	return 0;
}
module_init(proc_uboot_version_init);

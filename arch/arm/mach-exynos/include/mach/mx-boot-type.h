#ifndef __MX_BOOT_TYPES_H
#define __MX_BOOT_TYPES_H

#if defined(CONFIG_MX_SERIAL_TYPE) || defined(CONFIG_MX2_SERIAL_TYPE)
#define ATAG_BOOTINFO	0x5441000A
#ifdef CONFIG_MX2_SERIAL_TYPE
#define UBOOT_VERSION_SIZE		16
#define PART_INFO_SIZE			16
#else
#define UBOOT_VERSION_SIZE		10
#define PART_INFO_SIZE			7
#endif

struct tag_uboot_version {
	char uboot_version[UBOOT_VERSION_SIZE];
};

typedef struct tag_partinfo {
	u32 start_sec;
	u32 sec_num;
} tag_partinfo_t, tag_partinfo_p_t;

struct tag_bootinfo {
#ifdef CONFIG_MX2_SERIAL_TYPE
	unsigned int uboot_magic_number;
#endif
	/* if (signed_check == 1); then ckeck */
	unsigned int signed_check;
	/* mark for lost */
	unsigned int lost_mark;
	/* uboot svn version */
	unsigned int uboot_svn_version;
	/* Eng, Release, Oversea */
	char uboot_release_version[UBOOT_VERSION_SIZE];	/* this is the minimum size */
	tag_partinfo_t partinfo[PART_INFO_SIZE];
};

extern struct tag_bootinfo bootinfo;
extern char saved_uboot_version[UBOOT_VERSION_SIZE];
#endif//CONFIG_MX_SERIAL_TYPE/CONFIG_MX2_SERIAL_TYPE
#endif//__MX_BOOT_TYPES_H
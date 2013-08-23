#ifndef _LINUX_MX_LOGO_H
#define _LINUX_MX_LOGO_H

/*
 *  meizu logo to be displayed on boot
 */

#include <linux/linux_logo.h>

struct meizu_logo {
	int type;			/* one of LINUX_LOGO_* */
	unsigned int width;
	unsigned int height;
	unsigned int xoffset;
	unsigned int yoffset;
	unsigned int clutsize;		/* LINUX_LOGO_CLUT224 only */
	unsigned int bgd;			/* background color*/	
	const unsigned char *clut;	/* LINUX_LOGO_CLUT224 only */
	const unsigned char *data;
};

struct meizu_cmap {
	unsigned int start;			/* First entry	*/
	unsigned int len;			/* Number of entries */
	unsigned char *red;			/* Red values	*/
	unsigned char *green;
	unsigned char *blue;
	unsigned char *transp;			/* transparency, can be NULL */
};

#endif//_LINUX_MX_LOGO_H
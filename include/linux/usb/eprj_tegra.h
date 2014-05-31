/*
 * Copyright (c) 2011-2013, EternityProject Development.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef __EPRJ_TEGRA_H
#define __EPRJ_TEGRA_H

enum tegra_connect_type {
#ifdef CONFIG_MACH_ENDEAVORU
	CONNECT_TYPE_CLEAR = -2,
	CONNECT_TYPE_UNKNOWN = -1,
#endif
	CONNECT_TYPE_NONE = 0,
	CONNECT_TYPE_SDP,
	CONNECT_TYPE_DCP,
	CONNECT_TYPE_CDP,
	CONNECT_TYPE_NV_CHARGER,
	CONNECT_TYPE_NON_STANDARD_CHARGER
#ifdef CONFIG_MACH_ENDEAVORU
	,CONNECT_TYPE_INTERNAL
#endif
};

#endif

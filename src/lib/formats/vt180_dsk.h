// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/*********************************************************************

    formats/vt180_dsk.h

    DEC VT180 (VT18X) disk format

*********************************************************************/
#ifndef MAME_FORMATS_VT180_DSK_H
#define MAME_FORMATS_VT180_DSK_H

#pragma once

#include "wd177x_dsk.h"

class vt180_format : public wd177x_format
{
public:
	vt180_format();

	virtual const char *name() const noexcept override;
	virtual const char *description() const noexcept override;
	virtual const char *extensions() const noexcept override;

private:
	static const format formats[];
};

extern const vt180_format FLOPPY_VT180_FORMAT;

#endif // MAME_FORMATS_VT180_DSK_H

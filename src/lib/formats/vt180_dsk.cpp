// license:BSD-3-Clause
// copyright-holders:Adam Di Carlo
/*********************************************************************

    formats/vt180_dsk.cpp

    DEC VT180 (VT18X) disk format

    The VT18X controller takes single-sided 48 tpi 5.25" drives (RX180)
    and records MFM only (EK-VT18X-TM-001 6.6).  A disk has 40 tracks of
    nine 512-byte sectors numbered 1 to 9: the CP/M boot sector reads
    sectors 1-9 of each track.  Images are the sectors in order, track 0
    first.

    The track layout is what the VT180 formatter (DDFMT.COM) writes with
    a WRITE TRACK, the same on every track: 80 bytes of 4E, an index
    address mark, 50 of 4E, then sectors 1 to 9 in order with no
    interleave, each with a 22-byte gap 2 and a 65-byte gap 3.  DDFMT's
    gap 3 is one 00 then 64 4E; it's all 4E here, which a read can't
    tell apart.  DDFMT fills the sectors with E5.

*********************************************************************/

#include "formats/vt180_dsk.h"

vt180_format::vt180_format() : upd765_format(formats)
{
}

const char *vt180_format::name() const noexcept
{
	return "vt180";
}

const char *vt180_format::description() const noexcept
{
	return "DEC VT180 disk image";
}

const char *vt180_format::extensions() const noexcept
{
	return "img";
}

const vt180_format::format vt180_format::formats[] = {
	{
		floppy_image::FF_525, floppy_image::SSDD, floppy_image::MFM,
		2000, // 2us, 300rpm
		9, 40, 1,
		512, {},
		1, {},
		80, 50, 22, 65
	},
	{}
};

const vt180_format FLOPPY_VT180_FORMAT;

//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2005-2017, Thierry Lelegard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
//----------------------------------------------------------------------------
//
//  Video attributes for MPEG-1 and MPEG-2.
//
//----------------------------------------------------------------------------

#include "tsVideoAttributes.h"
#include "tsFormat.h"



//----------------------------------------------------------------------------
// Convert to a string object
//----------------------------------------------------------------------------

ts::VideoAttributes::operator std::string () const
{
    if (!_is_valid) {
        return "";
    }

    std::string desc (Format ("%" FMT_SIZE_T "dx%" FMT_SIZE_T "d", _hsize, _vsize));
    if (_progressive) {
        desc += 'p';
    }
    if (_interlaced) {
        desc += 'i';
    }
    desc += ", ";
    desc += frameRateName();
    desc += ", ";
    desc += aspectRatioName();
    desc += ", ";
    desc += chromaFormatName();

    return desc;
}


//----------------------------------------------------------------------------
// Get frame rate as a string
//----------------------------------------------------------------------------

std::string ts::VideoAttributes::frameRateName() const
{
    size_t fr100;

    if (!_is_valid || _fr_div == 0) {
        return "";
    }
    else if ((fr100 = frameRate100()) % 100 == 0) {
        return Format ("@%d Hz", int (fr100 / 100));
    }
    else {
        return Format ("@%d.%02d Hz", int (fr100 / 100), int (fr100 % 100));
    }
}


//----------------------------------------------------------------------------
// Refresh mode (both can be false if unspecifed)
//----------------------------------------------------------------------------

std::string ts::VideoAttributes::refreshModeName() const
{
    if (!_is_valid) {
        return "";
    }
    else if (_progressive) {
        return "progressive";
    }
    else if (_interlaced) {
        return "interlaced";
    }
    else {
        return "";
    }
}


//----------------------------------------------------------------------------
// Extract frame rate fields from frame rate code
//----------------------------------------------------------------------------

size_t ts::VideoAttributes::FRNum (uint8_t code)
{
    switch (code) {
        case 1:  return 24000;
        case 2:  return 24;
        case 3:  return 25;
        case 4:  return 30000;
        case 5:  return 30;
        case 6:  return 50;
        case 7:  return 60000;
        case 8:  return 60;
        default: return 0;
    }
}

size_t ts::VideoAttributes::FRDiv (uint8_t code)
{
    switch (code) {
        case 1:  return 1001;
        case 2:  return 1;
        case 3:  return 1;
        case 4:  return 1001;
        case 5:  return 1;
        case 6:  return 1;
        case 7:  return 1001;
        case 8:  return 1;
        default: return 1;
    }
}


//----------------------------------------------------------------------------
// Provides a video unit, starting with a 00 00 01 xx start code.
// Return true if the VideoAttributes object becomes valid or
// has new values.
//----------------------------------------------------------------------------

bool ts::VideoAttributes::moreBinaryData (const void* udata, size_t size)
{
    const uint8_t* data = reinterpret_cast <const uint8_t*> (udata);

    // Check start code
    if (size < 4 || data[0] != 0 || data[1] != 0 || data[2] != 1) {
        // Not a valid start code
        return false;
    }
    else if (data[3] == PST_SEQUENCE_HEADER && size >= 12) {
        // First set of value
        _sh_hsize = (GetUInt16 (data + 4) >> 4) & 0x0FFF;
        _sh_vsize = GetUInt16 (data + 5) & 0x0FFF;
        _sh_ar_code = (data[7] >> 4) & 0x0F;
        _sh_fr_code = data[7] & 0x0F;
        uint32_t fields = GetUInt32 (data + 8);
        _sh_bitrate = (fields >> 14) & 0x0003FFFF;
        _sh_vbv_size = (fields >> 3) & 0x000003FF;

        // Not yet complete, wait for next unit
        _waiting = true;
        return false;
    }
    else if (!_waiting) {
        // Not an interesting unit
        return false;
    }
    else if (data[3] == PST_EXTENSION && size >= 10) {
        // Extension data for MPEG-2
        // Extract fields:
        bool progressive = (data[5] & 0x08) != 0;
        bool interlaced = !progressive;
        uint8_t cf_code = (data[5] >> 1) & 0x03;
        size_t hsize_ext = (GetUInt16 (data + 5) >> 7) & 0x0003;
        size_t vsize_ext = (data[6] >> 5) & 0x03;
        BitRate bitrate_ext = (GetUInt16 (data + 6) >> 1) & 0x0FFF;
        size_t vbv_ext = data[8];
        size_t fr_ext_n = (data[9] >> 5) & 0x03;
        size_t fr_ext_d = data[9] & 0x1F;

        // Compute final values:
        size_t hsize = _sh_hsize | (hsize_ext << 12);
        size_t vsize = _sh_vsize | (vsize_ext << 12);
        size_t fr_num = FRNum (_sh_ar_code);
        size_t fr_div = FRDiv (_sh_ar_code);
        if (fr_num == 0) {
            // Not a valid aspect ratio code
            fr_num = size_t (_sh_ar_code) * (fr_ext_n + 1);
            fr_div = fr_ext_d + 1;
        }
        BitRate bitrate = _sh_bitrate | (bitrate_ext << 18);
        size_t vbv_size = _sh_vbv_size | (vbv_ext << 10);

        // Check modification
        bool changed = !_is_valid || _hsize != hsize || _vsize != vsize ||
            _ar_code != _sh_ar_code || _progressive != progressive ||
            _interlaced != interlaced || _cf_code != cf_code ||
            _fr_num != fr_num || _fr_div != fr_div || _bitrate != bitrate ||
            _vbv_size != vbv_size;

        // Commit final values
        _hsize = hsize;
        _vsize = vsize;
        _ar_code = _sh_ar_code;
        _progressive = progressive;
        _interlaced = interlaced;
        _cf_code = cf_code;
        _fr_num = fr_num;
        _fr_div = fr_div;
        _bitrate = bitrate;
        _vbv_size = vbv_size;

        _waiting = false;
        _is_valid = true;
        return changed;
    }
    else {
        // No extension data after sequence header => MPEG-1
        size_t fr_num = FRNum (_sh_ar_code);
        size_t fr_div = FRDiv (_sh_ar_code);
        bool changed = !_is_valid || _hsize != _sh_hsize || _vsize != _sh_vsize ||
            _ar_code != _sh_ar_code || _progressive || _interlaced || _cf_code != 0 ||
            _fr_num != fr_num || _fr_div != fr_div || _bitrate != _sh_bitrate ||
            _vbv_size != _sh_vbv_size;

        _hsize = _sh_hsize;
        _vsize = _sh_vsize;
        _ar_code = _sh_ar_code;
        _progressive = false;
        _interlaced = false;
        _cf_code = 0;
        _fr_num = fr_num;
        _fr_div = fr_div;
        _bitrate = _sh_bitrate;
        _vbv_size = _sh_bitrate;

        _waiting = false;
        _is_valid = true;
        return changed;
    }
}
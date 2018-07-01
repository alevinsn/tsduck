//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2005-2018, Thierry Lelegard
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
//  Transport stream processor shared library:
//  Output to HiDes modulator devices.
//
//----------------------------------------------------------------------------

#include "tsPlugin.h"
#include "tsPluginRepository.h"
#include "tsHiDesDevice.h"
#include "tsModulation.h"
#include "tsCOM.h"
TSDUCK_SOURCE;


//----------------------------------------------------------------------------
// Plugin definition
//----------------------------------------------------------------------------

namespace ts {
    class HiDesOutput: public OutputPlugin
    {
    public:
        // Implementation of plugin API
        HiDesOutput(TSP*);
        virtual bool start() override;
        virtual bool stop() override;
        virtual bool send(const TSPacket*, size_t) override;
        virtual bool isRealTime() override {return true;}
        virtual BitRate getBitrate() override;

    private:
        COM                 _com;         // COM initialization helper
        int                 _dev_number;  // Device adapter number.
        UString             _dev_name;    // Device name.
        bool                _set_gain;    // Adjust output gain.
        int                 _gain;        // Requested output gain in dB.
        TunerParametersDVBT _params;      // Tuning parameters.
        BitRate             _bitrate;     // Nominal output bitrate.
        HiDesDevice         _device;      // HiDes device object.
        HiDesDeviceInfo     _dev_info;    // HiDes device information.

        // Inaccessible operations
        HiDesOutput() = delete;
        HiDesOutput(const HiDesOutput&) = delete;
        HiDesOutput& operator=(const HiDesOutput&) = delete;
    };
}

TSPLUGIN_DECLARE_VERSION
TSPLUGIN_DECLARE_OUTPUT(hides, ts::HiDesOutput)


//----------------------------------------------------------------------------
// Constructor
//----------------------------------------------------------------------------

ts::HiDesOutput::HiDesOutput(TSP* tsp_) :
    OutputPlugin(tsp_, u"Send packets to a HiDes modulator device", u"[options]"),
    _com(*tsp_),
    _dev_number(-1),
    _dev_name(),
    _set_gain(false),
    _gain(0),
    _params(),
    _bitrate(0),
    _device(),
    _dev_info()
{
    option(u"adapter",   'a', UNSIGNED);
    option(u"bandwidth", 'b', Enumeration({
        {u"5", BW_5_MHZ},
        {u"6", BW_5_MHZ},
        {u"7", BW_7_MHZ},
        {u"8", BW_8_MHZ},
    }));
    option(u"constellation", 'c', Enumeration({
        {u"QPSK",   QPSK},
        {u"16-QAM", QAM_16},
        {u"64-QAM", QAM_64},
    }));
    option(u"device",         'd', STRING);
    option(u"frequency",      'f', POSITIVE);
    option(u"gain",            0,  INT32);
    option(u"guard-interval", 'g', Enumeration({
        {u"1/32", GUARD_1_32},
        {u"1/16", GUARD_1_16},
        {u"1/8",  GUARD_1_8},
        {u"1/4",  GUARD_1_4},
    }));
    option(u"high-priority-fec", 'h', Enumeration({
        {u"1/2", FEC_1_2},
        {u"2/3", FEC_2_3},
        {u"3/4", FEC_3_4},
        {u"5/6", FEC_5_6},
        {u"7/8", FEC_7_8},
    }));
    option(u"spectral-inversion", 's', SpectralInversionEnum);
    option(u"transmission-mode",  't', Enumeration({
        {u"2K", TM_2K},
        {u"4K", TM_4K},
        {u"8K", TM_8K},
    }));

    setHelp(u"Options:\n"
            u"\n"
            u"  -a value\n"
            u"  --adapter value\n"
            u"      Specify the HiDes adapter number to use. By default, the first HiDes\n"
            u"      device is selected. Use the command tshides to list all HiDes devices.\n"
            u"\n"
            u"  -b value\n"
            u"  --bandwidth value\n"
            u"      Bandwidth in MHz. Must be one of " + optionNames(u"bandwidth") + ".\n"
            u"      The default is 8 MHz.\n"
            u"\n"
            u"  -c value\n"
            u"  --constellation value\n"
            u"      Constellation type. Must be one of " + optionNames(u"constellation") + ".\n"
            u"      The default is 64-QAM.\n"
            u"\n"
            u"  -d name\n"
            u"  --device name\n"
            u"      Specify the HiDes device name to use. By default, the first HiDes device\n"
            u"      is selected. Use the command tshides to list all HiDes devices.\n"
            u"\n"
            u"  -f value\n"
            u"  --frequency value\n"
            u"      Frequency, in Hz, of the output carrier. There is no default.\n"
            u"\n"
            u"  --gain value\n"
            u"      Adjust the output gain to the specified value in dB.\n"
            u"\n"
            u"  -g value\n"
            u"  --guard-interval value\n"
            u"      Guard interval. Must be one of " + optionNames(u"guard-interval") + ".\n"
            u"      The default is 1/32.\n"
            u"\n"
            u"  --help\n"
            u"      Display this help text.\n"
            u"\n"
            u"  --h value\n"
            u"  --high-priority-fec value\n"
            u"      Error correction for high priority streams. Must be one of " + optionNames(u"high-priority-fec") + ".\n"
            u"      The default is 2/3.\n"
            u"\n"
            u"  --s value\n"
            u"  --spectral-inversion value\n"
            u"      Spectral inversion. Must be one of " + optionNames(u"spectral-inversion") + ".\n"
            u"      The default is auto.\n"
            u"\n"
            u"  -t value\n"
            u"  --transmission-mode value\n"
            u"      Transmission mode. Must be one of " + optionNames(u"transmission-mode") + ".\n"
            u"      The default is 8K.\n"
            u"\n"
            u"  --version\n"
            u"      Display the version number.\n");
}


//----------------------------------------------------------------------------
// Output start method
//----------------------------------------------------------------------------

bool ts::HiDesOutput::start()
{
    // Check that COM was correctly initialized
    if (!_com.isInitialized()) {
        tsp->error(u"COM initialization failure");
        return false;
    }
    if (_device.isOpen()) {
        tsp->error(u"already started");
        return false;
    }

    // Get options.
    _dev_number = intValue<int>(u"adapter", -1);
    _dev_name = value(u"device");
    _set_gain = present(u"gain");
    _gain = intValue<int>(u"gain");
    _params.bandwidth = enumValue<BandWidth>(u"bandwidth", BW_8_MHZ);
    _params.modulation = enumValue<Modulation>(u"constellation", QAM_64);
    _params.frequency = intValue<uint64_t>(u"frequency", 0);
    _params.guard_interval = enumValue<GuardInterval>(u"guard-interval", GUARD_1_32);
    _params.fec_hp = enumValue<InnerFEC>(u"high-priority-fec", FEC_2_3);
    _params.inversion = enumValue<SpectralInversion>(u"spectral-inversion", SPINV_AUTO);
    _params.transmission_mode = enumValue<TransmissionMode>(u"transmission-mode", TM_8K);

    // Check option consistency.
    if (_dev_number < 0 && _dev_name.empty()) {
        // Use first device by default.
        _dev_number = 0;
    }
    else if (_dev_number >= 0 && !_dev_name.empty()) {
        tsp->error(u"specify either HiDes adapter number or device name but not both");
        return false;
    }
    if (_params.frequency == 0) {
        tsp->error(u"no carrier frequency specified");
        return false;
    }

    // Nominal output bitrate is computed from the modulation parameters.
    _bitrate = _params.theoreticalBitrate();

    // Open the device, either by number or by name.
    if (_dev_number >= 0 && !_device.open(_dev_number, *tsp)) {
        return false;
    }
    if (!_dev_name.empty() && !_device.open(_dev_name, *tsp)) {
        return false;
    }
    if (!_device.getInfo(_dev_info, *tsp)) {
        _device.close(*tsp);
        return false;
    }
    tsp->verbose(u"using device %s", {_dev_info.toString()});

    // Tune to frequency.
    if (!_device.tune(_params, *tsp)) {
        _device.close(*tsp);
        return false;
    }

    // Adjust output gain if required.
    if (_set_gain) {
        int gain = _gain;
        if (!_device.setGain(gain, *tsp)) {
            _device.close(*tsp);
            return false;
        }
        // The value of gain is updated to effective value.
        tsp->verbose(u"adjusted output gain, requested %d dB, set to %d dB", {_gain, gain});
    }

    // Start transmission.
    if (!_device.startTransmission(*tsp)) {
        _device.close(*tsp);
        return false;
    }

    // Now fully ready to transmit.
    return true;
}


//----------------------------------------------------------------------------
// Output stop method
//----------------------------------------------------------------------------

bool ts::HiDesOutput::stop()
{
    return _device.stopTransmission(*tsp) && _device.close(*tsp);
}


//----------------------------------------------------------------------------
// Bitrate computation method
//----------------------------------------------------------------------------

ts::BitRate ts::HiDesOutput::getBitrate()
{
    // Was computed once, during start().
    return _bitrate;
}


//----------------------------------------------------------------------------
// Output method
//----------------------------------------------------------------------------

bool ts::HiDesOutput::send(const TSPacket* pkt, size_t packet_count)
{
    return _device.send(pkt, packet_count, *tsp, tsp);
}

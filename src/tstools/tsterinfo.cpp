//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2005-2019, Thierry Lelegard
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
//  DVB-T (terrestrial) information utility
//
//----------------------------------------------------------------------------

#include "tsMain.h"
#include "tsHFBand.h"
#include "tsBitrateDifferenceDVBT.h"
TSDUCK_SOURCE;
TS_MAIN(MainCode);


//----------------------------------------------------------------------------
//  Lists of possible real values for DVB-T modulation parameters
//  (exclude "auto" and "unspecified" values).
//----------------------------------------------------------------------------

namespace {

    const ts::Enumeration DVBTModulationEnum({
        {u"QPSK",   ts::QPSK},
        {u"16-QAM", ts::QAM_16},
        {u"64-QAM", ts::QAM_64},
    });

    const ts::Enumeration DVBTHPFECEnum({
        {u"1/2",  ts::FEC_1_2},
        {u"2/3",  ts::FEC_2_3},
        {u"3/4",  ts::FEC_3_4},
        {u"5/6",  ts::FEC_5_6},
        {u"7/8",  ts::FEC_7_8},
    });

    const ts::Enumeration DVBTBandWidthEnum({
        {u"8-MHz", ts::BW_8_MHZ},
        {u"7-MHz", ts::BW_7_MHZ},
        {u"6-MHz", ts::BW_6_MHZ},
        {u"5-MHz", ts::BW_5_MHZ},
    });

    const ts::Enumeration DVBTGuardIntervalEnum({
        {u"1/32", ts::GUARD_1_32},
        {u"1/16", ts::GUARD_1_16},
        {u"1/8",  ts::GUARD_1_8},
        {u"1/4",  ts::GUARD_1_4},
    });
}


//----------------------------------------------------------------------------
//  Command line options
//----------------------------------------------------------------------------

class Options: public ts::Args
{
    TS_NOBUILD_NOCOPY(Options);
public:
    Options(int argc, char *argv[]);
    virtual ~Options();

    uint64_t          frequency;     // Carrier frequency from which to get UHF channel
    uint32_t          uhf_channel;   // UHF channel from which to compute frequency
    uint32_t          vhf_channel;   // VHF channel from which to compute frequency
    int32_t           hf_offset;     // UHF/VHF offset from channel
    ts::BitRate       bitrate;       // TS bitrate from which to guess modulation parameters
    size_t            max_guess;     // Max number of modulation parameters to guess.
    ts::Modulation    constellation; // Modulation parameters to compute bitrate
    ts::InnerFEC      fec_hp;
    ts::GuardInterval guard_interval;
    ts::BandWidth     bandwidth;
    bool              simple;          // Simple output
    bool              default_region;  // Display the default region for UHF/VHF band frequency layout
    ts::UString       hfband_region;   // Region for UHF/VHF band frequency layout
};

// Destructor.
Options::~Options() {}

// Constructor.
Options::Options(int argc, char *argv[]) :
    Args(u"Compute or convert DVB-Terrestrial information", u"[options]"),
    frequency(0),
    uhf_channel(0),
    vhf_channel(0),
    hf_offset(0),
    bitrate(0),
    max_guess(0),
    constellation(ts::QAM_AUTO),
    fec_hp(ts::FEC_NONE),
    guard_interval(ts::GUARD_AUTO),
    bandwidth(ts::BW_AUTO),
    simple(false),
    default_region(false),
    hfband_region()
{
    option(u"bandwidth", 'w', DVBTBandWidthEnum);
    help(u"bandwidth", u"Specify the OFMD bandwith, used to compute the resulting bitrate.");

    option(u"bitrate", 'b', UINT32);
    help(u"bitrate",
         u"Transport stream bitrate in b/s, based on 188-byte packets. Given this "
         u"bitrate, tsterinfo will try to guess the OFDM modulation parameters.");

    option(u"constellation", 'c', DVBTModulationEnum);
    help(u"constellation", u"Specify the OFMD constellation, used to compute the resulting bitrate.");

    option(u"default-region", 'd');
    help(u"default-region",
         u"Display the default region for UHF/VHF band frequency layout.");

    option(u"frequency", 'f', UNSIGNED);
    help(u"frequency", u"Carrier frequency in Hz. UHF or VHF channel and offset will be displayed.");

    option(u"guard-interval", 'g', DVBTGuardIntervalEnum);
    help(u"guard-interval",
         u"Specify the OFMD guard interval, used to compute the resulting bitrate.");

    option(u"high-priority-fec", 'h', DVBTHPFECEnum);
    help(u"high-priority-fec",
         u"Specify the OFMD error correction for high priority streams, "
         u"used to compute the resulting bitrate.");

    option(u"max-guess", 'm', POSITIVE);
    help(u"max-guess",
         u"When used with --bitrate, specify the maximum number of modulation "
         u"parameters sets to display. By default, display one set of parameters, "
         u"the one giving the closest bitrate.");

    option(u"offset-count", 'o', INTEGER, 0, 1, -10, 10);
    help(u"offset-count",
         u"Specify the number of offsets from the UHF or VHF channel. The default "
         u"is zero. See options --uhf-channel and --vhf-channel.");

    option(u"hf-band-region", 'r', STRING);
    help(u"hf-band-region", u"name",
         u"Specify the region for UHF/VHF band frequency layout.");

    option(u"simple", 's');
    help(u"simple",
         u"Produce simple output: only numbers, no comment, typically useful "
         u"to write scripts.");

    option(u"uhf-channel", 'u', POSITIVE);
    help(u"uhf-channel",
         u"Specify the UHF channel number of the carrier. Can be combined with an "
         u"--offset-count option. The resulting frequency will be displayed.");

    option(u"vhf-channel", 'v', POSITIVE);
    help(u"vhf-channel",
         u"Specify the VHF channel number of the carrier. Can be combined with an "
         u"--offset-count option. The resulting frequency will be displayed.");

    analyze(argc, argv);

    frequency      = intValue<uint64_t>(u"frequency", 0);
    uhf_channel    = intValue<uint32_t>(u"uhf-channel", 0);
    vhf_channel    = intValue<uint32_t>(u"vhf-channel", 0);
    hf_offset      = intValue<int32_t>(u"offset-count", 0);
    bitrate        = intValue<ts::BitRate>(u"bitrate", 0);
    max_guess      = intValue<ts::BitRate>(u"max-guess", 1);
    constellation  = enumValue(u"constellation", ts::QAM_64);
    fec_hp         = enumValue(u"high-priority-fec", ts::FEC_AUTO);
    guard_interval = enumValue(u"guard-interval", ts::GUARD_AUTO);
    bandwidth      = enumValue(u"bandwidth", ts::BW_8_MHZ);
    simple         = present(u"simple");
    default_region = present(u"default-region");
    hfband_region  = value(u"hf-band-region");

    if ((fec_hp == ts::FEC_AUTO && guard_interval != ts::GUARD_AUTO) ||
        (fec_hp != ts::FEC_AUTO && guard_interval == ts::GUARD_AUTO))
    {
        error(u"specify either both --guard-interval and --high-priority-fec value or none");
    }

    exitOnError();
}


//----------------------------------------------------------------------------
//  This routine displays a name/value pair
//----------------------------------------------------------------------------

namespace {
    void Display(const ts::UString& name, const ts::UString& value, const ts::UString& unit)
    {
        std::cout << "  " << name.toJustified(value, 37, u'.', 1) << " " << unit << std::endl;
    }
}


//----------------------------------------------------------------------------
//  Program entry point
//----------------------------------------------------------------------------

int MainCode(int argc, char *argv[])
{
    Options opt(argc, argv);

    // Get UHF/VHF frequency layout.
    ts::HFBand::SetDefaultRegion(opt.hfband_region);
    const ts::HFBand* uhf = ts::HFBand::GetBand(opt.hfband_region, ts::HFBand::UHF, opt);
    const ts::HFBand* vhf = ts::HFBand::GetBand(opt.hfband_region, ts::HFBand::VHF, opt);

    // Display the default region for UHF/VHF band frequency layout
    if (opt.default_region) {
        if (!opt.simple) {
            std::cout << "Default region for UHF/VHF: ";
        }
        std::cout << ts::HFBand::DefaultRegion(opt) << std::endl;
    }

    // Convert UHF channel to frequency
    if (opt.uhf_channel > 0) {
        if (opt.uhf_channel < uhf->firstChannel() || opt.uhf_channel > uhf->lastChannel()) {
            std::cerr << ts::UString::Decimal(opt.uhf_channel)
                      << " is not a valid UHF channel, valid range is "
                      << ts::UString::Decimal(uhf->firstChannel()) << " - "
                      << ts::UString::Decimal(uhf->lastChannel()) << std::endl;
        }
        else if (opt.simple) {
            std::cout << uhf->frequency(opt.uhf_channel, opt.hf_offset) << std::endl;
        }
        else {
            std::cout << "Carrier Frequency: "
                      << ts::UString::Decimal(uhf->frequency(opt.uhf_channel, opt.hf_offset))
                      << " Hz" << std::endl;
        }
    }

    // Convert VHF channel to frequency
    if (opt.vhf_channel > 0) {
        if (opt.vhf_channel < vhf->firstChannel() || opt.vhf_channel > vhf->lastChannel()) {
            std::cerr << ts::UString::Decimal(opt.vhf_channel)
                      << " is not a valid VHF channel, valid range is "
                      << ts::UString::Decimal(vhf->firstChannel()) << " - "
                      << ts::UString::Decimal(vhf->lastChannel()) << std::endl;
        }
        else if (opt.simple) {
            std::cout << vhf->frequency(opt.vhf_channel, opt.hf_offset) << std::endl;
        }
        else {
            std::cout << "Carrier Frequency: "
                      << ts::UString::Decimal(vhf->frequency(opt.vhf_channel, opt.hf_offset))
                      << " Hz" << std::endl;
        }
    }

    // Convert frequency to UHF/VHF channel
    if (opt.frequency > 0) {
        if (uhf->inBand(opt.frequency)) {
            if (opt.simple) {
                std::cout << uhf->channelNumber(opt.frequency) << std::endl
                          << uhf->offsetCount(opt.frequency) << std::endl;
            }
            else {
                int channel = uhf->channelNumber(opt.frequency);
                int offset = uhf->offsetCount(opt.frequency);
                std::cout << "UHF channel: " << channel << ", offset: " << offset << std::endl;
                uint64_t exact_freq = uhf->frequency(channel, offset);
                int diff = int(int64_t(opt.frequency) - int64_t(exact_freq));
                if (::abs(diff) > 1) {
                    std::cout << "Warning: exact frequency for channel "
                              << channel << ", offset " << offset << " is "
                              << ts::UString::Decimal(exact_freq) << " Hz, differ by "
                              << ts::UString::Decimal(diff) << " Hz" << std::endl;
                }
            }
        }
        else if (vhf->inBand(opt.frequency)) {
            if (opt.simple) {
                std::cout << vhf->channelNumber(opt.frequency) << std::endl
                          << vhf->offsetCount(opt.frequency) << std::endl;
            }
            else {
                int channel = vhf->channelNumber(opt.frequency);
                int offset = vhf->offsetCount(opt.frequency);
                std::cout << "VHF channel: " << channel << ", offset: " << offset << std::endl;
                uint64_t exact_freq = vhf->frequency(channel, offset);
                int diff = int(int64_t(opt.frequency) - int64_t(exact_freq));
                if (::abs(diff) > 1) {
                    std::cout << "Warning: exact frequency for channel "
                              << channel << ", offset " << offset << " is "
                              << ts::UString::Decimal(exact_freq) << " Hz, differ by "
                              << ts::UString::Decimal(diff) << " Hz" << std::endl;
                }
            }
        }
        else {
            std::cerr << ts::UString::Decimal(opt.frequency) << " Hz is not in UHF or VHF bands (VHF: "
                      << ts::UString::Decimal(vhf->lowestFrequency()) << " - "
                      << ts::UString::Decimal(vhf->highestFrequency()) << ", UHF: "
                      << ts::UString::Decimal(uhf->lowestFrequency()) << " - "
                      << ts::UString::Decimal(uhf->highestFrequency()) << ")"
                      << std::endl;
        }
    }

    // Compute TS bitrate from modulation parameters
    if (opt.fec_hp != ts::FEC_AUTO && opt.guard_interval != ts::GUARD_AUTO) {
        ts::ModulationArgs params;
        params.delivery_system = ts::DS_DVB_T;
        params.bandwidth = opt.bandwidth;
        params.fec_hp = opt.fec_hp;
        params.modulation = opt.constellation;
        params.guard_interval = opt.guard_interval;
        if (opt.simple) {
            std::cout << params.theoreticalBitrate() << std::endl;
        }
        else {
            std::cout << "Transport stream bitrate: "
                      << ts::UString::Decimal(params.theoreticalBitrate())
                      << " b/s" << std::endl;
        }
    }

    // Guess possible modulation parameters from bitrate
    if (opt.bitrate > 0) {

        // Build a list of all possible modulation parameters for this bitrate.
        ts::BitrateDifferenceDVBTList params_list;
        ts::BitrateDifferenceDVBT::EvaluateToBitrate(params_list, opt.bitrate);

        // Display all relevant parameters, up to max_guess
        // (in case of equal differences, display them all)
        int last_diff = 0;
        size_t count = 0;
        for (auto it = params_list.begin();
             it != params_list.end() && (count < opt.max_guess || std::abs(it->bitrate_diff) == std::abs(last_diff));
             ++it, ++count)
        {
            last_diff = it->bitrate_diff;
            if (opt.simple) {
                std::cout << it->tune.theoreticalBitrate() << std::endl
                          << ts::BandWidthEnum.name(it->tune.bandwidth.value()) << std::endl
                          << ts::InnerFECEnum.name(it->tune.fec_hp.value()) << std::endl
                          << ts::ModulationEnum.name(it->tune.modulation.value()) << std::endl
                          << ts::GuardIntervalEnum.name(it->tune.guard_interval.value()) << std::endl;
            }
            else {
                if (count > 0) {
                    std::cout << std::endl;
                }
                Display(u"Nominal bitrate", ts::UString::Decimal(it->tune.theoreticalBitrate()), u"b/s");
                Display(u"Bitrate difference", ts::UString::Decimal(it->bitrate_diff), u"b/s");
                Display(u"Bandwidth", ts::BandWidthEnum.name(it->tune.bandwidth.value()), u"");
                Display(u"FEC (high priority)", ts::InnerFECEnum.name(it->tune.fec_hp.value()), u"");
                Display(u"Constellation", ts::ModulationEnum.name(it->tune.modulation.value()), u"");
                Display(u"Guard interval", ts::GuardIntervalEnum.name(it->tune.guard_interval.value()), u"");
            }
        }
    }

    return opt.valid() ? EXIT_SUCCESS : EXIT_FAILURE;
}

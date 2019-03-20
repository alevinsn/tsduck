#include "tsPlugin.h"
#include "tsPluginRepository.h"
#include "tsService.h"
#include "tsSectionDemux.h"
#include "tsCyclingPacketizer.h"
//#include "tsEITProcessor.h"
#include "tsPAT.h"
#include "tsPMT.h"
//#include "tsCAT.h"
#include "tsSDT.h"
#include "tsSpliceInformationTable.h"

using namespace ts;

#include <utility>
#include <set>
//#include <deque>
using namespace std;


//----------------------------------------------------------------------------
// Plugin definition
//----------------------------------------------------------------------------

class SwitchProgramPlugin : public ProcessorPlugin, private TableHandlerInterface, private SectionHandlerInterface
{
public:
    // Implementation of plugin API
    SwitchProgramPlugin(TSP*);
    virtual bool getOptions() override;
    virtual bool start() override;
    virtual Status processPacket(TSPacket&, bool&, bool&) override;

private:

    // Private data
    bool m_bAbort;
    SectionDemux m_demux;
    CyclingPacketizer m_pzer_pat;       // Packetizer for modified PAT
    CyclingPacketizer m_pzer_sdt;       // Packetizer for modified SDT
    CyclingPacketizer m_pzer_pmt;       // Packetizer for modified PMT
    Service m_primaryService;
    Service m_alternateService;
    std::set<uint32_t> m_eventIDSet;
    uint32_t m_nAudioStreamIndex;

    pair<ts::PID, ts::PID> m_videoPIDPair;
    set<ts::PID> m_videoPIDSet;
    pair<ts::PID, ts::PID> m_audioPIDPair;
    set<ts::PID> m_audioPIDSet;
    pair<ts::PID, ts::PID> m_eventPIDPair;
    set<ts::PID> m_eventPIDSet;
    set<ts::PID> m_alwaysDropPIDSet;
    //set<ts::PID> m_otherProgramPIDSet;

    //set<ts::PID> m_pidsWithExtraPackets;

    bool m_bProcessedPMTs;

    virtual void handleTable(SectionDemux&, const BinaryTable&) override;
    virtual void handleSection(SectionDemux& demux, const Section& section) override;

    // Process specific tables
    void processPAT(PAT&);
    void processPMT(PMT&, bool bIsAlternate);
    void processSDT(SDT&);

    // ------------------------------------------------------------
    // Data Model
    // ------------------------------------------------------------

    // In case of splicing by component, each PID in the service is identified by a component tag.
    // This is a map of component tags, indexed by PID.
    typedef std::map<ts::PID, uint8_t> TagByPID;

    // A reduced form of splice event.
    class Event
    {
    public:
        bool     out;  // When true, this is a "splice out" event, "splice in" otherwise.
        uint32_t id;   // Splice event id, in case of cancelation later.

                       // Constructor
        Event(bool out_ = false, uint32_t id_ = 0) : out(out_), id(id_) {}
    };

    // Each PID of the service has a list of splice events, sorted by PTS value.
    // For simplicity, we use a map, indexed by PTS value.
    // If several events have the same PTS, the last one prevails.
    typedef std::map<uint64_t, Event> EventByPTS;

    // State of a PID which is subject to splicing.
    class PIDState
    {
    public:
        ts::PID    pid;                  // PID value.
        uint8_t    cc;                   // Last continuity counter in the PID.
        bool       currentlyOut;         // PID is currently spliced out.
        uint64_t   outStart;             // When spliced out, PTS value at the time of splicing out.
        uint64_t   totalAdjust;          // Total removed time in PTS units.
        uint64_t   lastPTS;              // Last PTS value in this PID.
        EventByPTS events;               // Ordered map of upcoming slice events.
        bool       immediateOut;         // Currently splicing out for an immediate event
        uint32_t   immediateEventId;     // Event ID associated with immediate splice out event
        bool       cancelImmediateOut;   // Want to cancel current immediate splice out event
        bool       isAudio;              // Associated with audio stream
        bool       isVideo;              // Associated with video stream
        uint64_t   lastOutEnd;           // When spliced back in, PTS value at the time of the splice in
        uint64_t   ptsLastSeekPoint;     // PTS of last seek point for this PID
        uint64_t   ptsBetweenSeekPoints; // PTS difference between last seek points for this PID
        bool       isAlternate;
        //deque<deque<TSPacket> > m_droppedPacketsDeque;

                                         // Constructor
        PIDState(ts::PID = PID_NULL);

        // Add a splicing event in a PID.
        void addEvent(uint64_t pts, bool spliceOut, uint32_t eventId, bool immediate);
        void addEvent(const SpliceInsert& cmd, const TagByPID& tags);

    };

    // All PID's in the service are described by a map, indexed by PID.
    typedef std::map<ts::PID, PIDState> StateByPID;

    TagByPID           _tagsByPID;   // Mapping between PID's and component tags in the service.
    StateByPID         _states;      // Map of current state by PID in the service.

    // Inaccessible operations
    SwitchProgramPlugin() = delete;
    SwitchProgramPlugin(const SwitchProgramPlugin&) = delete;
    SwitchProgramPlugin& operator=(const SwitchProgramPlugin&) = delete;
};

TSPLUGIN_DECLARE_VERSION
TSPLUGIN_DECLARE_PROCESSOR(switchprogram, SwitchProgramPlugin)

SwitchProgramPlugin::PIDState::PIDState(ts::PID pid_) :
    pid(pid_),
    cc(0xFF),
    currentlyOut(false),
    outStart(INVALID_PTS),
    totalAdjust(0),
    lastPTS(INVALID_PTS),
    events(),
    immediateOut(false),
    immediateEventId(0),
    cancelImmediateOut(false),
    isAudio(false),
    isVideo(false),
    lastOutEnd(INVALID_PTS),
    ptsLastSeekPoint(INVALID_PTS),
    ptsBetweenSeekPoints(INVALID_PTS)
{
}

//----------------------------------------------------------------------------
// Constructor
//----------------------------------------------------------------------------

SwitchProgramPlugin::SwitchProgramPlugin(TSP* tsp_) :
    ProcessorPlugin(tsp_, u"Switch to alternate program based on event ID", u"[options] service"),
    m_bAbort(false),
    m_nAudioStreamIndex(0),
    m_demux(this, this),
    m_pzer_pat(PID_PAT, CyclingPacketizer::ALWAYS),
    m_pzer_sdt(PID_SDT, CyclingPacketizer::ALWAYS),
    m_pzer_pmt(PID_NULL, CyclingPacketizer::ALWAYS),
    m_bProcessedPMTs(false)
{
    option(u"primary-service-id", 0, UINT16);
    help(u"primary-service-id", u"Specify the id for the primary service.");

    option(u"alt-service-id", 0, UINT16);
    help(u"alt-service-id", u"Specify the id for the alternate service.");

    option(u"audio-stream-index", 0, UINT32);
    help(u"audio-stream-index", u"Specifies index of audio stream to use for each program.");

    option(u"event-id", 0, INTEGER, 0, UNLIMITED_COUNT, 0, 100000);
    //option(u"event-id", 0, INTEGER, 0, UNLIMITED_COUNT, 0, UNLIMITED_VALUE);
    help(u"event-id",
         u"Switch to other program when encountering specific event IDs. Multiple --event-id options "
         u"may be specified.");
}


//----------------------------------------------------------------------------
// Get options method
//----------------------------------------------------------------------------

bool SwitchProgramPlugin::getOptions()
{
    m_primaryService.setId(intValue<uint16_t>(u"primary-service-id"));
    m_alternateService.setId(intValue<uint16_t>(u"alt-service-id"));

    m_nAudioStreamIndex = intValue<uint32_t>(u"audio-stream-index", 0, 0);
    getIntValues(m_eventIDSet, u"event-id");

    return true;
}


//----------------------------------------------------------------------------
// Start method
//----------------------------------------------------------------------------

bool SwitchProgramPlugin::start()
{
    // Initialize the demux
    m_demux.reset();
    m_demux.addPID(PID_SDT);
    m_demux.addPID(PID_PAT);

    // Reset other states
    m_bAbort = false;

    m_pzer_pat.reset();
    m_pzer_sdt.reset();
    m_pzer_pmt.reset();

    m_pzer_pmt.setPID(PID_NULL);

    return true;
}


//----------------------------------------------------------------------------
// Invoked by the demux when a complete table is available.
//----------------------------------------------------------------------------

void SwitchProgramPlugin::handleTable(SectionDemux& demux, const BinaryTable& table)
{
    switch (table.tableId())
    {
        case TID_PAT:
            if (table.sourcePID() == PID_PAT)
            {
                PAT pat(table);
                if (pat.isValid())
                    processPAT(pat);
            }
            break;

        case TID_SDT_ACT:
            if (table.sourcePID() == PID_SDT)
            {
                SDT sdt(table);
                if (sdt.isValid())
                    processSDT(sdt);
            }
            break;

        case TID_PMT:
        {
            PMT pmt(table);
            if (pmt.isValid())
            {
                if (m_primaryService.hasId(pmt.service_id))
                    processPMT(pmt, false);
                else if (m_alternateService.hasId(pmt.service_id))
                {
                    processPMT(pmt, true);
                    // never want to keep PMT packets from alternate program
                    m_alwaysDropPIDSet.insert(m_alternateService.getPMTPID());
                }

                if (m_videoPIDSet.size() == 2)
                {
                    m_bProcessedPMTs = true;
                }
            }
            break;
        }

        default:
            break;
    }
}

void SwitchProgramPlugin::processSDT(SDT& sdt)
{
    if (sdt.services.find(m_primaryService.getId()) == sdt.services.end())
    {
        m_bAbort = true;
        return;
    }

    if (sdt.services.find(m_alternateService.getId()) == sdt.services.end())
    {
        m_bAbort = true;
        return;
    }

    sdt.services.erase(m_alternateService.getId());

    // Replace the SDT.in the PID
    m_pzer_pat.removeSections(TID_SDT_ACT, sdt.ts_id);
    m_pzer_pat.addTable(sdt);
}

void SwitchProgramPlugin::processPAT(PAT& pat)
{
    PAT::ServiceMap::iterator it = pat.pmts.find(m_primaryService.getId());

    if (it == pat.pmts.end())
    {
        tsp->error(u"Primary service id 0x%X not found in PAT", { m_primaryService.getId() });
        m_bAbort = true;
        return;
    }

    if (!m_primaryService.hasPMTPID(it->second))
    {
        m_primaryService.setPMTPID(it->second);
        m_demux.addPID(it->second);

        tsp->verbose(u"found primary service id 0x%X, PMT PID is 0x%X", { m_primaryService.getId(), m_primaryService.getPMTPID() });

        m_pzer_pmt.setPID(it->second);
    }

    it = pat.pmts.find(m_alternateService.getId());

    if (it == pat.pmts.end())
    {
        tsp->error(u"Alternate service id 0x%X not found in PAT", {m_alternateService.getId()});
        m_bAbort = true;
        return;
    }

    if (!m_alternateService.hasPMTPID(it->second))
    {
        m_alternateService.setPMTPID(it->second);
        m_demux.addPID(it->second);

        tsp->verbose(u"found alternate service id 0x%X, PMT PID is 0x%X", {m_alternateService.getId(), m_alternateService.getPMTPID()});
    }

    pat.pmts.erase(m_alternateService.getId());

    // Replace the PAT.in the PID
    m_pzer_pat.removeSections(TID_PAT);
    m_pzer_pat.addTable(pat);
}


//----------------------------------------------------------------------------
//  This method processes a Program Map Table (PMT).
//----------------------------------------------------------------------------

void SwitchProgramPlugin::processPMT(PMT& pmt, bool bIsAlternate)
{
    ts::PID videoPID = PID_NULL;
    ts::PID firstAudioPID = PID_NULL;
    ts::PID secondAudioPID = PID_NULL;
    ts::PID eventPID = PID_NULL;

    PMT::StreamMap::const_iterator it;
    for (it = pmt.streams.begin(); it != pmt.streams.end(); it++)
    {
        const PMT::Stream& strm = it->second;
        const ts::PID& pid = it->first;
        if ((videoPID == PID_NULL) && strm.isVideo())
            videoPID = pid;
        else if ((firstAudioPID == PID_NULL) && strm.isAudio())
            firstAudioPID = pid;
        else if ((secondAudioPID == PID_NULL) && strm.isAudio())
            secondAudioPID = pid;
        else if ((eventPID == PID_NULL) && (strm.stream_type == ST_SCTE35_SPLICE))
            eventPID = pid;

        if (strm.isAudio() || strm.isVideo() || (strm.stream_type == ST_SCTE35_SPLICE))
        {
            PIDState pidState(pid);
            pidState.isAudio = strm.isAudio();
            pidState.isVideo = strm.isVideo();
            pidState.currentlyOut = bIsAlternate;
            pidState.isAlternate = bIsAlternate;
            _states.insert(std::make_pair(pid, pidState));

            // Look for an optional stream_identifier_descriptor for this component.
            uint8_t ctag = 0;
            if (strm.getComponentTag(ctag))
            {
                // We have found a component tag for this PID.
                _tagsByPID[pid] = ctag;
            }
        }
    }

    m_videoPIDSet.insert(videoPID);
    m_audioPIDSet.insert(firstAudioPID);
    m_audioPIDSet.insert(secondAudioPID);
    m_eventPIDSet.insert(eventPID);

    if (!bIsAlternate)
    {
        m_videoPIDPair.first = videoPID;
        if (m_nAudioStreamIndex == 0)
        {
            m_audioPIDPair.first = firstAudioPID;
            m_alwaysDropPIDSet.insert(secondAudioPID);
        }
        else
        {
            m_audioPIDPair.first = secondAudioPID;
            m_alwaysDropPIDSet.insert(firstAudioPID);
        }
        m_eventPIDPair.first = eventPID;

        m_demux.addPID(eventPID);

        // remove unused audio PID from PMT
        if (m_nAudioStreamIndex == 0)
            pmt.streams.erase(secondAudioPID);
        else
            pmt.streams.erase(firstAudioPID);
        // Replace the PMT.in the PID
        m_pzer_pmt.removeSections(TID_PMT, m_primaryService.getId());
        m_pzer_pmt.addTable(pmt);
    }
    else
    {
        m_videoPIDPair.second = videoPID;
        if (m_nAudioStreamIndex == 0)
        {
            m_audioPIDPair.second = firstAudioPID;
            m_alwaysDropPIDSet.insert(secondAudioPID);
        }
        else
        {
            m_audioPIDPair.second = secondAudioPID;
            m_alwaysDropPIDSet.insert(firstAudioPID);
        }
        m_eventPIDPair.second = eventPID;
        
        // also add video, audio, and event PIDs to always drop set--will be switched out as needed
        m_alwaysDropPIDSet.insert(videoPID);
        m_alwaysDropPIDSet.insert(m_audioPIDPair.second);
        //m_otherProgramPIDSet.insert(videoPID);
        //m_otherProgramPIDSet.insert(m_audioPIDPair.second);
        m_alwaysDropPIDSet.insert(eventPID);

        //m_demux.addPID(eventPID);
    }
}

void SwitchProgramPlugin::handleSection(SectionDemux& demux, const Section& section)
{
    if (m_eventPIDSet.find(section.sourcePID()) == m_eventPIDSet.end())
        return;

    // Try to extract a SpliceInsert command from the section.
    SpliceInsert cmd;
    if (!SpliceInformationTable::ExtractSpliceInsert(cmd, section))
    {
        // Not the right table or command, just ignore it.
        return;
    }

    // Filter events by ids if --event-id was specified.
    if (!m_eventIDSet.empty() && m_eventIDSet.find(cmd.event_id) == m_eventIDSet.end())
    {
        return;
    }

    if (cmd.immediate) {
        // Add an immediate splice event, which doesn't have a PTS value and is handled differently than scheduled splice events.
        for (StateByPID::iterator it = _states.begin(); it != _states.end(); ++it) {
            bool splice_out = cmd.splice_out;
            if (it->second.isAlternate)
                splice_out = !splice_out;
            tsp->verbose(u"adding 'immediate' splice %s with event ID 0x%X (%d) on PID 0x%X (%d) at PTS %d (%.3f s)",
                {splice_out ? u"out" : u"in", cmd.event_id, cmd.event_id, it->second.pid, it->second.pid, it->second.lastPTS,
                double(it->second.lastPTS) / double(SYSTEM_CLOCK_SUBFREQ)});
            //if (!_dryRun) {
                it->second.addEvent(cmd, _tagsByPID);
            //}
        }
    }
    else {
        // Add a new (or repeated) splice event for a given PTS value.
        tsp->verbose(u"adding splice %s at PTS %s with event ID 0x%X (%d)", {cmd.splice_out ? u"out" : u"in", cmd.program_pts.toString(), cmd.event_id, cmd.event_id});
        //if (!_dryRun) {
            for (StateByPID::iterator it = _states.begin(); it != _states.end(); ++it) {
                it->second.addEvent(cmd, _tagsByPID);
            }
        //}
    }
}


//----------------------------------------------------------------------------
// Add a splicing event in a PID, basic form.
//----------------------------------------------------------------------------

void SwitchProgramPlugin::PIDState::addEvent(uint64_t pts, bool spliceOut, uint32_t eventId, bool immediate)
{
    if (immediate) {
        if (isAlternate)
            spliceOut = !spliceOut;

        // Ignore immediate splice in events if not coupled with a prior splice out event
        // In addition, only support a single event ID at a time--if currently splicing out for a
        // particular event ID and receive a splice immediate event for another event ID, disregard it
        if (immediateOut) {
            if (!spliceOut && (immediateEventId == eventId)) {
                cancelImmediateOut = true;
            }
        }
        else if (spliceOut) {
            immediateOut = true;
            immediateEventId = eventId;
            cancelImmediateOut = false;
        }
    }
    else {
        // Ignore invalid PTS or PTS from the past, before last PTS value in this PID.
        // Note that the initial "lastPTS" of a PID is an invalid value, indicating "not yet available".
        if (pts <= PTS_DTS_MASK && (lastPTS > PTS_DTS_MASK || SequencedPTS(lastPTS, pts))) {
            // Simply replace the event if it already existed.
            events[pts] = Event(spliceOut, eventId);
        }
    }
}


//----------------------------------------------------------------------------
// Add a splicing event in a PID, from a SpliceInsert command.
//----------------------------------------------------------------------------

void SwitchProgramPlugin::PIDState::addEvent(const SpliceInsert& cmd, const TagByPID& tags)
{
    uint64_t pts = 0;

    if (!cmd.immediate) {
        if (cmd.program_splice && cmd.program_pts.set()) {
            // Same PTS value for all components in the service.
            pts = cmd.program_pts.value();
        }
        else {
            // There is one PTS value per service component in the command, search our PTS value.
            const TagByPID::const_iterator it1 = tags.find(pid);
            const SpliceInsert::SpliceByComponent::const_iterator it2 =
                it1 == tags.end() ?                     // no component tag found for our PID
                cmd.components_pts.end() :              // so there won't be any PTS
                cmd.components_pts.find(it1->second);   // search PTS value for the component type
            if (it2 == cmd.components_pts.end() || !it2->second.set()) {
                // The SpliceInsert does not specify any PTS for our PID, nothing to do.
                return;
            }
            else {
                pts = it2->second.value();
            }
        }
    }

    // Add the splicing event.
    addEvent(pts, cmd.splice_out, cmd.event_id, cmd.immediate);

    // If this is a "splice out" event with "auto return", then also add the "splice in" event at the end of the sequence.
    if (cmd.splice_out && cmd.use_duration && cmd.auto_return) {
        addEvent((pts + cmd.duration_pts) & PTS_DTS_MASK, false, cmd.event_id, cmd.immediate);
    }
}


//----------------------------------------------------------------------------
// Packet processing method
//----------------------------------------------------------------------------

ProcessorPlugin::Status SwitchProgramPlugin::processPacket(TSPacket& pkt, bool& flush, bool& bitrate_changed)
{
    const ts::PID pid = pkt.getPID();

    // Filter interesting sections
    m_demux.feedPacket(pkt);

    // If a fatal error occurred during section analysis, give up.
    if (m_bAbort) {
        return TSP_END;
    }

    // As long as the original service-id or PMT are unknown, nullify packets
    if (!m_bProcessedPMTs)
        return TSP_NULL;

    if (pid == 802)
        OutputDebugStringA("We got here 3.\n");

    // Replace packets using packetizers
    if (pid == PID_PAT)
        m_pzer_pat.getNextPacket(pkt);
    else if (pid == PID_SDT)
        m_pzer_sdt.getNextPacket(pkt);
    else if (pid == m_primaryService.getPMTPID())
        m_pzer_pmt.getNextPacket(pkt);

    bool bSplicePoint = pkt.getRandomAccessIndicator();
    /*
    if (bSplicePoint)
    {
        uint64_t currentPTS = pkt.getPTS();

        if (m_videoPIDSet.find(pid) != m_videoPIDSet.end())
        {
            tsp->verbose(u"Video splice point on PID 0x%X (%d) at PTS %d (%.3f s)",
            {pid, pid, currentPTS, (double) currentPTS / (double) SYSTEM_CLOCK_SUBFREQ});
        }
        else if (m_audioPIDSet.find(pid) != m_audioPIDSet.end())
        {
            tsp->verbose(u"Audio splice point on PID 0x%X (%d) at PTS %d (%.3f s)",
            {pid, pid, currentPTS, (double) currentPTS / (double) SYSTEM_CLOCK_SUBFREQ});
        }
    }
    */

    bool bFirst = true;
    bool bVideo = false;
    bool bAudio = false;
    bool bEvent = false;

    if (m_videoPIDSet.find(pid) != m_videoPIDSet.end())
    {
        bVideo = true;
        if (pid != m_videoPIDPair.first)
            bFirst = false;
    }
    else if (m_audioPIDSet.find(pid) != m_audioPIDSet.end())
    {
        bAudio = true;
        if (pid != m_audioPIDPair.first)
            bFirst = false;
    }
    else if (m_eventPIDSet.find(pid) != m_eventPIDSet.end())
    {
        bEvent = true;
        if (pid != m_eventPIDPair.first)
            bFirst = false;
    }

    if (!bVideo && !bAudio && !bEvent)
    {
        if (m_alwaysDropPIDSet.find(pid) != m_alwaysDropPIDSet.end())
            return TSP_DROP;

        return TSP_OK;
    }

    StateByPID::iterator itFirst;
    StateByPID::iterator itSecond;
    if (bVideo)
    {
        itFirst = _states.find(m_videoPIDPair.first);
        itSecond = _states.find(m_videoPIDPair.second);
    }
    else if (bAudio)
    {
        itFirst = _states.find(m_audioPIDPair.first);
        itSecond = _states.find(m_audioPIDPair.second);
    }
    else if (bEvent)
    {
        // use video states then
        itFirst = _states.find(m_eventPIDPair.first);
        itSecond = _states.find(m_eventPIDPair.second);
    }

    if ((itFirst == _states.end()) || (itSecond == _states.end()))
        return TSP_OK;

    PIDState& firstState(itFirst->second);
    PIDState& secondState(itSecond->second);

    PIDState& state = _states[pid];

    if (bVideo || bAudio)
    {
        // If this packet has a PTS, there is maybe a splice point to process.
        if (pkt.hasPTS()) {
            // Keep last PTS of the PID.
            uint64_t currentPTS = pkt.getPTS();
            if (bSplicePoint)
            {
                // keep track of time between seek points
                // this time is used for determining which audio seek point is closest to the
                // video splice out time when handling immediate splice events
                if (state.ptsLastSeekPoint != INVALID_PTS) {
                    state.ptsBetweenSeekPoints = currentPTS - state.ptsLastSeekPoint;
                }

                state.ptsLastSeekPoint = currentPTS;
            }
            state.lastPTS = currentPTS;
        }
    }

    // doing this doesn't help--the times pretty much never align
    /*
    if (m_otherProgramPIDSet.find(pid) != m_otherProgramPIDSet.end())
    {
        PIDState& otherState = bFirst ? secondState : firstState;
        if (otherState.immediateOut && !otherState.currentlyOut)
        {
            // then other state needs to be switched out
            // temporarily archive packets from this PID, because we might need them when it is time to
            // splice in on this PID
            // use deque of deques, which each sub-deque being an in-order array of packets
            // that correspond to a single closed GOP/etc (but potentially an incomplete set)
            if (bSplicePoint)
            {
                deque<TSPacket> newDeque;
                newDeque.push_back(pkt);
                state.m_droppedPacketsDeque.push_back(newDeque);
            }
            else
            {
                if (!state.m_droppedPacketsDeque.empty())
                    state.m_droppedPacketsDeque.back().push_back(pkt);
            }
        }

        return TSP_DROP;
    }
    */

    if (m_alwaysDropPIDSet.find(pid) != m_alwaysDropPIDSet.end())
        return TSP_DROP;

    if (bVideo || bAudio)
    {
        if (state.immediateOut && !state.currentlyOut)
        {
            if (bSplicePoint)
            {
                bool doSpliceOut = true;
                ts::PID currentVideoPID = bFirst ? m_videoPIDPair.first : m_videoPIDPair.second;

                if (state.isAudio && currentVideoPID != PID_NULL) {
                    PIDState& videoState = _states[currentVideoPID];
                    if (!videoState.currentlyOut) {
                        doSpliceOut = false;
                    }
                    else if (state.lastPTS < videoState.outStart) {
                        if ((state.ptsBetweenSeekPoints == INVALID_PTS) ||
                            ((videoState.outStart - state.lastPTS) > (state.ptsBetweenSeekPoints / 2))) {
                            doSpliceOut = false;
                        }
                    }
                }

                if (doSpliceOut) {
                    state.currentlyOut = true;
                    state.outStart = state.lastPTS;

                    tsp->verbose(u"Immediate splice out on PID 0x%X (%d) at PTS %d (%.3f s)",
                    {pid, pid, state.lastPTS, (double) state.lastPTS / (double) SYSTEM_CLOCK_SUBFREQ});

                    m_alwaysDropPIDSet.insert(pid);
                    //m_otherProgramPIDSet.insert(pid);

                    if (bVideo)
                    {
                        if (bFirst)
                        {
                            m_alwaysDropPIDSet.erase(m_videoPIDPair.second);
                            //m_otherProgramPIDSet.erase(m_videoPIDPair.second);
                            m_alwaysDropPIDSet.erase(m_eventPIDPair.second);
                            m_alwaysDropPIDSet.insert(m_eventPIDPair.first);
                            // don't bother adjusting currentOut for event PID states--not used
                        }
                        else
                        {
                            m_alwaysDropPIDSet.erase(m_videoPIDPair.first);
                            //m_otherProgramPIDSet.erase(m_videoPIDPair.first);
                            m_alwaysDropPIDSet.erase(m_eventPIDPair.first);
                            m_alwaysDropPIDSet.insert(m_eventPIDPair.second);
                        }
                    }
                    else
                    {
                        if (bFirst)
                        {
                            m_alwaysDropPIDSet.erase(m_audioPIDPair.second);
                            //m_otherProgramPIDSet.erase(m_audioPIDPair.second);
                        }
                        else
                        {
                            m_alwaysDropPIDSet.erase(m_audioPIDPair.first);
                            //m_otherProgramPIDSet.erase(m_audioPIDPair.first);
                        }
                    }

                    /*
                    PIDState& otherState = bFirst ? secondState : firstState;
                    while (!otherState.m_droppedPacketsDeque.empty())
                    {
                        // iterate through each deque, find first PTS that is at least greater than
                        // or equal to outStart
                        // if less than, discard entire sub-deque because need to splice in on splice points
                        deque<TSPacket>& oldestDeque = otherState.m_droppedPacketsDeque.front();
                        TSPacket& splicePointPkt = oldestDeque.front();
                        uint64_t pts = splicePointPkt.getPTS();
                        if (pts < state.outStart)
                        {
                            otherState.m_droppedPacketsDeque.pop_front();
                            continue;
                        }

                        // replace the packet with the contents of the replacement packet
                        pkt = splicePointPkt;
                        oldestDeque.pop_front();
                        if (oldestDeque.empty())
                            otherState.m_droppedPacketsDeque.pop_front();

                        if (!m_pidsWithExtraPackets.empty())
                            m_pidsWithExtraPackets.insert(otherState.pid);

                        return TSP_OK;
                    }
                    */
                }
            }
        }

        if (m_alwaysDropPIDSet.find(pid) != m_alwaysDropPIDSet.end())
            return TSP_DROP;

        if (state.currentlyOut)
        {
            // need to splice in on the other program
            if (bSplicePoint)
            {
                /*
                bool doSpliceIn = true;

                ts::PID currentVideoPID = bFirst ? m_videoPIDPair.first : m_videoPIDPair.second;

                if (state.isAudio && currentVideoPID != PID_NULL) {
                    PIDState& videoState = _states[currentVideoPID];
                    if (videoState.currentlyOut) {
                        doSpliceIn = false;
                    }
                    else if (state.lastPTS < videoState.lastOutEnd) {
                        if ((state.ptsBetweenSeekPoints == INVALID_PTS) ||
                            ((videoState.lastOutEnd - state.lastPTS) > (state.ptsBetweenSeekPoints / 2))) {
                            doSpliceIn = false;
                        }
                    }
                }
                */

                PIDState& otherState = bFirst ? secondState : firstState;
                if (state.lastPTS >= otherState.outStart)
                {
                    // can splice back in at this point
                    state.cancelImmediateOut = false;
                    state.immediateOut = false;
                    state.immediateEventId = 0;
                    state.currentlyOut = false;

                    // Splicing back in, restarting the transmission of the PID.
                    // Add removed period to the total removed time (in PTS units).
                    if (state.outStart != INVALID_PTS) {
                        state.totalAdjust = (state.totalAdjust + (state.lastPTS - state.outStart)) & PTS_DTS_MASK;
                        state.outStart = INVALID_PTS;
                    }

                    state.lastOutEnd = state.lastPTS;

                    tsp->verbose(u"Immediate splice in on PID 0x%X (%d) at PTS %d (%.3f s)",
                    {pid, pid, state.lastPTS, (double) state.lastPTS / (double) SYSTEM_CLOCK_SUBFREQ});
                }
            }
        }

        if (state.currentlyOut)
            return TSP_DROP;
    }

    if (!bFirst)
    {
        // need to switch the PID to that of the first program
        if (state.isVideo)
            pkt.setPID(m_videoPIDPair.first);
        else if (state.isAudio)
            pkt.setPID(m_audioPIDPair.first);
        else
            pkt.setPID(m_eventPIDPair.first);
    }

    // need to use continuity counter associated with stream from primary program, as that is the one that is
    // used
    PIDState& ccstate = firstState;

    // Fix continuity counters.
    if (ccstate.cc != 0xFF)
    {
        pkt.setCC((ccstate.cc + 1) & CC_MASK);
    }
    ccstate.cc = pkt.getCC();

    return TSP_OK;
}

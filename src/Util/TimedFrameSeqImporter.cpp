/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "TimedFrameSeqImporter.h"
#include "gettext.h"

using namespace cnoid;

std::string TimedFrameSeqImporter::non_timed_frame_seq_message()
{
    return _("The seq data to import is not a timed-frame seq data");
}

std::string TimedFrameSeqImporter::invalid_frame_size_message()
{
    return _("Invalid frame size.");
}

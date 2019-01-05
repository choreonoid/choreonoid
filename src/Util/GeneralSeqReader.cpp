/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "GeneralSeqReader.h"
#include <fmt/format.h>
#include "gettext.h"

using namespace cnoid;
using fmt::format;

std::string GeneralSeqReader::mismatched_seq_type_message(const std::string& type, AbstractSeq* seq)
{
    return format(_("Seq type \"{0}\" cannot be load into {1}."), type, seq->seqType());
}

std::string GeneralSeqReader::has_frame_time_unsupported_message(double formatVersion)
{
    return format(_("The \"hasFrameTime\" parameter is not availabe in format version {}."), formatVersion);
}

std::string GeneralSeqReader::unkown_frame_rate_for_time_frame_seq_message()
{
    return _("The frame rate for importing a timed-frame seq is not specified.");
}

std::string GeneralSeqReader::frames_key_not_found_message()
{
    return _("The \"frames\" key is not found.");
}

std::string GeneralSeqReader::no_frame_data_message()
{
    return _("The \"frames\" key does not contain any frame data.");
}

std::string GeneralSeqReader::invalid_num_parts_messaage()
{
    return _("Invaid number of parts is specified");
}

std::string GeneralSeqReader::invalid_frame_size_message()
{
    return _("Invalid frame size.");
}

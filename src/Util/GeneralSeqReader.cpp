/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "GeneralSeqReader.h"
#include <boost/format.hpp>
#include "gettext.h"

using namespace cnoid;
using boost::format;

std::string GeneralSeqReader::mismatched_seq_type_message(const std::string& type, AbstractSeq* seq)
{
    return str(format(_("Seq type \"%1%\" cannot be load into %2%.")) % type % seq->seqType());
}

std::string GeneralSeqReader::has_frame_time_unsupported_message(double formatVersion)
{
    return str(format(_("The \"hasFrameTime\" parameter is not availabe in format version %1%.")) % formatVersion);
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

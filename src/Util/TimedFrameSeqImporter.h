/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "ValueTree.h"
#include "AbstractSeq.h"
#include <functional>
#include <type_traits>

extern void* enabler;

namespace cnoid {

class TimedFrameSeqImporter
{
    static std::string non_timed_frame_seq_message();
    static std::string invalid_frame_size_message();
    
public:
    template<
      class SeqType,
      typename std::enable_if<
          std::is_base_of<AbstractSeq, SeqType>::value &&
          !std::is_base_of<AbstractMultiSeq, SeqType>::value>::type *& = enabler
    >
    void import(
        const Mapping& archive, SeqType& seq, std::ostream& os, 
        std::function<void(const Listing& v, typename SeqType::value_type& value)> readValue)
    {
        if(!archive.get("hasFrameTime", false)){
            archive.throwException(non_timed_frame_seq_message());
        }
        seq.checkSeqType(archive);
        seq.readSeqContent(archive);

        seq.setNumFrames(0);
        
        const Listing& frames = *archive.findListing("frames");
        if(frames.isValid()){
            const int numFrames = frames.size();
            for(int i=0; i < numFrames; ++i){
                const Listing& v = *frames[i].toListing();
                double time = v[0].toDouble();
                int frameIndex = seq.frameOfTime(time);
                if(frameIndex >= seq.numFrames()){
                    seq.setNumFrames(frameIndex + 1, true);
                }
                typename SeqType::value_type& value = seq[frameIndex];
                readValue(v, value);
            }
        }
    }

    template<
      class SeqType,
      typename std::enable_if<std::is_base_of<AbstractMultiSeq, SeqType>::value>::type *& = enabler
    >
    void import(
        const Mapping& archive, SeqType& seq, std::ostream& os,
        std::function<void(const ValueNode& node, typename SeqType::value_type& v)> readValue)
    {
        if(!archive.get("hasFrameTime", false)){
            archive.throwException(non_timed_frame_seq_message());
        }
        seq.checkSeqType(archive);
        seq.readSeqContent(archive);

        const int numParts = seq.readNumParts(archive);
        seq.setDimension(0, numParts);
        
        const int frameSize = numParts + 1 /* +1 is for time value */;
    
        const Listing& frames = *archive.findListing("frames");
        if(frames.isValid()){
            const int nFrames = frames.size();
            for(int i=0; i < nFrames; ++i){
                const Listing& values = *frames[i].toListing();
                if(values.size() != frameSize){
                    values.throwException(invalid_frame_size_message());
                }
                double time = values[0].toDouble();
                int frameIndex = seq.frameOfTime(time);
                if(frameIndex >= seq.numFrames()){
                    seq.setNumFrames(frameIndex + 1, true);
                }
                typename SeqType::Frame v = seq.frame(frameIndex);
                for(int j=0; j < numParts; ++j){
                    readValue(values[j+1], v[j]);
                }
            }
        }
    }
};

}

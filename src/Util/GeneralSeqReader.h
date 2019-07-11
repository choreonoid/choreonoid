/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_GENERAL_SEQ_READER_H
#define CNOID_UTIL_GENERAL_SEQ_READER_H

#include "ValueTree.h"
#include "AbstractSeq.h"
#include <functional>
#include <type_traits>
#include <ostream>

extern void* enabler;

namespace cnoid {

class GeneralSeqReader
{
    static std::string mismatched_seq_type_message(const std::string& type, AbstractSeq* seq);
    static std::string has_frame_time_unsupported_message(double formatVersion);
    static std::string unkown_frame_rate_for_time_frame_seq_message();
    static std::string frames_key_not_found_message();
    static std::string no_frame_data_message();
    static std::string invalid_num_parts_messaage();
    static std::string invalid_frame_size_message();
    
    std::ostream& os_;
    const Mapping* archive_;
    AbstractSeq* seq_;
    double formatVersion_;
    bool hasFrameTime_;
    int numParts_;

    std::function<bool(GeneralSeqReader& reader, const std::string& type)> customSeqTypeChecker;

public:
    GeneralSeqReader(std::ostream& os)
        : os_(os)
    {
        archive_ = nullptr;
        seq_ = nullptr;
        formatVersion_ = 2.0;
        hasFrameTime_ = false;
        numParts_ = 0;
    }

    std::ostream& os() { return os_; }
    const Mapping* archive() { return archive_; }
    AbstractSeq* seq() { return seq_; }
    double formatVersion() const { return formatVersion_; }
    bool hasFrameTime() const { return hasFrameTime_; }
    int numParts() const { return numParts_; }

    bool checkSeqType(const std::string& type) const {
        return (type == seq_->seqType());
    };

    void setCustomSeqTypeChecker(std::function<bool(GeneralSeqReader& reader, const std::string& type)> func){
        customSeqTypeChecker = func;
    }
    
private:
    const Listing& getFrames(const Mapping* archive)
    {
        auto framesNode = archive->find("frames");
        if(!framesNode->isValid()){
            archive->throwException(frames_key_not_found_message());
        }
        const Listing& frames = *framesNode->toListing();
        if(frames.empty()){
            frames.throwException(no_frame_data_message());
        }
        return frames;
    }
        
public:        
    bool readHeaders(const Mapping* archive, AbstractSeq* seq)
    {
        archive_ = archive;
        seq_ = seq;
        
        formatVersion_ = archive->get("formatVersion", 1.0);

        auto& typeNode = (*archive)["type"];
        auto type = typeNode.toString();
        bool isTypeMatched;
        if(customSeqTypeChecker){
            isTypeMatched = customSeqTypeChecker(*this, type);
        } else {
            isTypeMatched = checkSeqType(type);
        }
        if(!isTypeMatched){
            typeNode.throwException(mismatched_seq_type_message(type, seq));
        }
        
        std::string content;
        if(archive->read("content", content)){
            seq->setSeqContentName(content);
        } else if(formatVersion_ < 2.0 && archive->read("purpose", content)){
            seq->setSeqContentName(content);
        }

        hasFrameTime_ = false;
        auto node_hasFrameTime = archive->find("hasFrameTime");
        if(node_hasFrameTime->isValid()){
            if(formatVersion_ < 2.0){
                node_hasFrameTime->throwException(has_frame_time_unsupported_message(formatVersion_));
            }
            hasFrameTime_ = node_hasFrameTime->toBool();
        }
        
        if(!hasFrameTime_){
            seq->setFrameRate(archive->read<double>("frameRate"));
        } else {
            if(seq->getFrameRate() <= 0){
                double r;
                if(archive->read("frameRate", r)){
                    seq->setFrameRate(r);
                } else {
                    os_ << unkown_frame_rate_for_time_frame_seq_message() << std::endl;
                    return false;
                }
            }
        }

        if(dynamic_cast<AbstractMultiSeq*>(seq)){
            auto& node = (*archive)["numParts"];
            numParts_ = node.toInt();
            if(numParts_ < 1){
                node.throwException(invalid_num_parts_messaage());
            }
        }

        return true;
    }

    template<
      class SeqType,
      typename std::enable_if<
          std::is_base_of<AbstractSeq, SeqType>::value &&
          !std::is_base_of<AbstractMultiSeq, SeqType>::value>::type *& = enabler
    >
    bool read(
        const Mapping* archive, SeqType* seq,
        std::function<void(const Listing& srcNode, int topIndex, typename SeqType::value_type& seqValue)> readValue)
    {
        return readHeaders(archive, seq) && readFrames(archive, seq, readValue);
    }

    template<
      class SeqType,
      typename std::enable_if<
          std::is_base_of<AbstractSeq, SeqType>::value &&
          !std::is_base_of<AbstractMultiSeq, SeqType>::value>::type *& = enabler
    >
    bool readFrames(
        const Mapping* archive, SeqType* seq,
        std::function<void(const Listing& srcNode, int topIndex, typename SeqType::value_type& seqValue)> readValue)
    {
        const Listing& frames = getFrames(archive);
        const int numFrames = frames.size();
        seq->setNumFrames(hasFrameTime_ ? 0 : numFrames);

        for(int i=0; i < numFrames; ++i){
            const Listing& srcValue = *frames[i].toListing();
            if(!hasFrameTime_){
                auto& seqValue = (*seq)[i];
                readValue(srcValue, 0, seqValue);
            } else {
                double time = srcValue[0].toDouble();
                int frameIndex = seq->frameOfTime(time);
                if(frameIndex >= seq->numFrames()){
                    seq->setNumFrames(frameIndex + 1, true);
                }
                auto& seqValue = (*seq)[frameIndex];
                readValue(srcValue, 1, seqValue);
            }
        }

        return true;
    }

    template<
      class SeqType,
      typename std::enable_if<std::is_base_of<AbstractMultiSeq, SeqType>::value>::type *& = enabler
    >
    bool read(
        const Mapping* archive, SeqType* seq,
        std::function<void(const ValueNode& srcNode, typename SeqType::value_type& seqValue)> readValue)
    {
        return readHeaders(archive, seq) && readFrames(archive, seq, readValue);
    }

    template<
      class SeqType,
      typename std::enable_if<std::is_base_of<AbstractMultiSeq, SeqType>::value>::type *& = enabler
    >
    bool readFrames(
        const Mapping* archive, SeqType* seq,
        std::function<void(const ValueNode& srcNode, typename SeqType::value_type& seqValue)> readValue)
    {
        int frameDataSize = numParts_;
        if(hasFrameTime_){
            frameDataSize += 1;
        }

        const Listing& frames = getFrames(archive);
        const int numFrames = frames.size();

        if(hasFrameTime_){
            seq->setDimension(0, numParts_);
        } else {
            seq->setDimension(numFrames, numParts_);
        }

        for(int i=0; i < numFrames; ++i){
            const Listing& srcValues = *frames[i].toListing();
            if(srcValues.size() != frameDataSize){
                srcValues.throwException(invalid_frame_size_message());
            }
            if(!hasFrameTime_){
                auto seqFrame = seq->frame(i);
                for(int j=0; j < numParts_; ++j){
                    readValue(srcValues[j], seqFrame[j]);
                }
            } else {
                double time = srcValues[0].toDouble();
                int frameIndex = seq->frameOfTime(time);
                if(frameIndex >= seq->numFrames()){
                    seq->setNumFrames(frameIndex + 1, true);
                }
                auto seqFrame = seq->frame(frameIndex);
                for(int j=0; j < numParts_; ++j){
                    readValue(srcValues[j+1], seqFrame[j]);
                }
            }
        }

        return true;
    }
};

}

#endif

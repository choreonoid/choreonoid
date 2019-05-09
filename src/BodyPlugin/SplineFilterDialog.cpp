/**
   @author Shin'ichiro Nakaoka
*/

#include "SplineFilterDialog.h"
#include <cnoid/EigenUtil>
#include <cnoid/UniformCubicBSpline>
#include <cnoid/MultiValueSeqItem>
#include <cnoid/MultiSE3SeqItem>
#include <cnoid/BodyMotionItem>
#include <cnoid/MultiVector3Seq>
#include <cnoid/ZMPSeq>
#include <cnoid/ItemList>
#include <cnoid/Archive>
#include <cnoid/MenuManager>
#include <cnoid/ItemTreeView>
#include <cnoid/MessageView>
#include <cnoid/Dialog>
#include <cnoid/Button>
#include <cnoid/SpinBox>
#include <cnoid/Array2D>
#include <QBoxLayout>
#include <QLabel>
#include <QDialogButtonBox>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

typedef MultiSeqItem<MultiVector3Seq> MultiVector3SeqItem;

typedef UniformCubicBSplineVector<Array2D<double>, Array2D<double>::Row, double> Spline;


void applySplineFilter
(AbstractSeq& seq, Array2D<double>& buf,
 double inputFrameRate, double outputFrameRate, double timeLengthRatio,
 std::function<void(int frame, Spline::SolutionType value)> setElement)
{
    timeLengthRatio *= (seq.getFrameRate() / inputFrameRate);
    const double timeLength = timeLengthRatio * seq.getTimeLength();

    int numFrames = timeLength * outputFrameRate;
    seq.setFrameRate(outputFrameRate);
    seq.setNumFrames(0);
    seq.setNumFrames(numFrames);

    Spline spline(buf, buf.rowSize(), buf.colSize(), inputFrameRate);

    const double dt = 1.0 / outputFrameRate;

    Spline::SolutionType solution;
    for(int i=0; i < numFrames; ++i){
        double time = i * dt;
        spline.calc(time / timeLengthRatio, solution);
        setElement(i, solution);
    }
}
    

void setMultiValueSeqElement(MultiValueSeq& seq, int frameIndex, Spline::SolutionType value)
{
    std::copy(&value(0), &value(0) + value.size(), seq.frame(frameIndex).begin());
}
    

void applySplineFilter
(MultiValueSeq& seq, double inputFrameRate, double outputFrameRate, double timeLengthRatio)
{
    const int n = seq.numFrames();
    Array2D<double> buf(n, seq.numParts());
    for(int i=0; i < n; ++i){
        MultiValueSeq::Frame frame = seq.frame(i);
        std::copy(frame.begin(), frame.end(), buf.row(i).begin());
    }

    applySplineFilter(
        seq, buf, inputFrameRate, outputFrameRate, timeLengthRatio,
        [&seq](int frameIndex, Spline::SolutionType value){
            setMultiValueSeqElement(seq, frameIndex, value);
        });
}
        

void setVector3SeqElement(Vector3Seq& seq, int frameIndex, Spline::SolutionType value)
{
    seq[frameIndex] << value(0), value(1), value(2);
}


void applySplineFilter
(Vector3Seq& seq, double inputFrameRate, double outputFrameRate, double timeLengthRatio)
{
    int n = seq.numFrames();
    Array2D<double> buf(n, 3);
    for(int i=0; i < n; ++i){
        const Vector3& v = seq[i];
        Array2D<double>::Row row = buf.row(i);
        row[0] = v[0];
        row[1] = v[1];
        row[2] = v[2];
    }
    applySplineFilter(
        seq, buf, inputFrameRate, outputFrameRate, timeLengthRatio,
        [&seq](int frameIndex, Spline::SolutionType value){
            setVector3SeqElement(seq, frameIndex, value);
        });
}


void setMultiVector3SeqElement(MultiVector3Seq& seq, int frameIndex, Spline::SolutionType value)
{
    MultiVector3Seq::Frame frame = seq.frame(frameIndex);
    int index = 0;
    for(int j=0; j < seq.numParts(); ++j){
        frame[j] << value(index), value(index+1), value(index+2);
        index += 3;
    }
}


void applySplineFilter
(MultiVector3Seq& seq, double inputFrameRate, double outputFrameRate, double timeLengthRatio)
{
    int numFrames = seq.numFrames();
    int numParts = seq.numParts();
    Array2D<double> buf(numFrames, numParts * 3);
    for(int i=0; i < numFrames; ++i){
        MultiVector3Seq::Frame frame = seq.frame(i);
        Array2D<double>::Row row = buf.row(i);
        int index = 0;
        for(int j=0; j < numParts; ++j){
            const Vector3& p = frame[j];
            row[index++] = p.x();
            row[index++] = p.y();
            row[index++] = p.z();
        }
    }
    applySplineFilter(
        seq, buf, inputFrameRate, outputFrameRate, timeLengthRatio,
        [&seq](int frameIndex, Spline::SolutionType value){
            setMultiVector3SeqElement(seq, frameIndex, value);
        });
}


void setMultiSE3SeqElement(MultiSE3Seq& seq, int frameIndex, Spline::SolutionType value)
{
    MultiSE3Seq::Frame frame = seq.frame(frameIndex);
    int index = 0;
    for(int j=0; j < seq.numParts(); ++j){
        SE3& x = frame[j];
        x.translation() << value(index), value(index+1), value(index+2);
        Vector3 rpy(value(index+3), value(index+4), value(index+5));
        x.rotation() = rotFromRpy(rpy);
        index += 6;
    }
}


void applySplineFilter
(MultiSE3Seq& seq, double inputFrameRate, double outputFrameRate, double timeLengthRatio)
{
    int numFrames = seq.numFrames();
    int numParts = seq.numParts();
    Array2D<double> buf(numFrames, numParts * 6);
    for(int i=0; i < numFrames; ++i){
        MultiSE3Seq::Frame frame = seq.frame(i);
        Array2D<double>::Row row = buf.row(i);
        int index = 0;
        for(int j=0; j < numParts; ++j){
            SE3& x = frame[j];
            Vector3& p = x.translation();
            row[index++] = p[0];
            row[index++] = p[1];
            row[index++] = p[2];
            Vector3 rpy(rpyFromRot(Matrix3(x.rotation())));
            row[index++] = rpy[0];
            row[index++] = rpy[1];
            row[index++] = rpy[2];
        }
    }
    applySplineFilter(
        seq, buf, inputFrameRate, outputFrameRate, timeLengthRatio,
        [&seq](int frameIndex, Spline::SolutionType value){
            setMultiSE3SeqElement(seq, frameIndex, value);
        });
}


class SplineFilterDialog : public Dialog
{
public:
    SplineFilterDialog();
    
    ostream& os;
    
    CheckBox inputFrameRateCheck;
    DoubleSpinBox inputFrameRateSpin;
    CheckBox outputFrameRateCheck;
    DoubleSpinBox outputFrameRateSpin;
    DoubleSpinBox timeLengthRatioSpin;
    
    bool store(Archive& archive);
    void restore(const Archive& archive);
    
    virtual void onAccepted();
};

}


void cnoid::initializeSplineFilterDialog(ExtensionManager* ext)
{
    static SplineFilterDialog* dialog = 0;

    if(!dialog){
        dialog = ext->manage(new SplineFilterDialog());

        ext->menuManager().setPath("/Filters").addItem(_("Spline filter"))->
            sigTriggered().connect([](){ dialog->show(); });

        ext->setProjectArchiver("SplineFilterDialog",
                                [](Archive& archive){ return dialog->store(archive); },
                                [](const Archive& archive){ dialog->restore(archive); });
    }
}

            
SplineFilterDialog::SplineFilterDialog()
    : os(MessageView::instance()->cout())
{
    setWindowTitle(_("Spline Filter"));

    QVBoxLayout* vbox = new QVBoxLayout();
    setLayout(vbox);

    QHBoxLayout* hbox = new QHBoxLayout();
    inputFrameRateCheck.setText(_("Input frame rate"));
    inputFrameRateCheck.setChecked(false);
    hbox->addWidget(&inputFrameRateCheck);

    inputFrameRateSpin.setDecimals(0);
    inputFrameRateSpin.setRange(1.0, 10000.0);
    inputFrameRateSpin.setSingleStep(1.0);
    inputFrameRateSpin.setValue(200.0);
    hbox->addWidget(&inputFrameRateSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    outputFrameRateCheck.setText(_("Output frame rate"));
    outputFrameRateCheck.setChecked(false);
    hbox->addWidget(&outputFrameRateCheck);

    outputFrameRateSpin.setDecimals(0);
    outputFrameRateSpin.setRange(1.0, 10000.0);
    outputFrameRateSpin.setSingleStep(1.0);
    outputFrameRateSpin.setValue(200.0);
    hbox->addWidget(&outputFrameRateSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("Time length ratio")));
    timeLengthRatioSpin.setDecimals(2);
    timeLengthRatioSpin.setRange(0.01, 999.99);
    timeLengthRatioSpin.setValue(1.0);
    hbox->addWidget(&timeLengthRatioSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    PushButton* applyButton = new PushButton(_("&Apply"));
    applyButton->setDefault(true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(applyButton, QDialogButtonBox::AcceptRole);
    applyButton->sigClicked().connect([this](){ accept(); });
    
    vbox->addWidget(buttonBox);
}


bool SplineFilterDialog::store(Archive& archive)
{
    archive.write("isInputFrameRateSpecified", inputFrameRateCheck.isChecked());
    archive.write("inputFrameRate", inputFrameRateSpin.value());
    archive.write("isOutputFrameRateSpecified", outputFrameRateCheck.isChecked());
    archive.write("outputFrameRate", outputFrameRateSpin.value());
    return true;
}


void SplineFilterDialog::restore(const Archive& archive)
{
    inputFrameRateCheck.setChecked(archive.get("isInputFrameRateSpecified", inputFrameRateCheck.isChecked()));
    inputFrameRateSpin.setValue(archive.get("inputFrameRate", inputFrameRateSpin.value()));
    outputFrameRateCheck.setChecked(archive.get("isOutputFrameRateSpecified", outputFrameRateCheck.isChecked()));
    outputFrameRateSpin.setValue(archive.get("outputFrameRate", outputFrameRateSpin.value()));
}


void SplineFilterDialog::onAccepted()
{
    double ratio = timeLengthRatioSpin.value();
        
    ItemList<AbstractMultiSeqItem> items = ItemTreeView::instance()->selectedItems<AbstractMultiSeqItem>();

    ItemList<MultiValueSeqItem> vItems = items;
    for(int i=0; i < vItems.size(); ++i){
        auto seq = vItems[i]->seq();
        double ifps = inputFrameRateCheck.isChecked()  ? inputFrameRateSpin.value()  : seq->frameRate();
        double ofps = outputFrameRateCheck.isChecked() ? outputFrameRateSpin.value() : seq->frameRate();
        
        os << format(_("Applying B-Spline filter to {0}: input frame rate = {1}, output frame rate = {2}"),
                     vItems[i]->name(), ifps, ofps) << endl;
        
        applySplineFilter(*seq, ifps, ofps, ratio);
    }

    ItemList<MultiVector3SeqItem> v3Items = items;
    for(int i=0; i < v3Items.size(); ++i){
        auto seq = v3Items[i]->seq();
        double ifps = inputFrameRateCheck.isChecked()  ? inputFrameRateSpin.value()  : seq->frameRate();
        double ofps = outputFrameRateCheck.isChecked() ? outputFrameRateSpin.value() : seq->frameRate();
        
        os << format(_("Applying B-Spline filter to {0}: input frame rate = {1}, output frame rate = {2}"),
                     v3Items[i]->name(), ifps, ofps) << endl;
        
        applySplineFilter(*seq, ifps, ofps, ratio);
    }

    ItemList<MultiSE3SeqItem> sItems = items;
    for(int i=0; i < sItems.size(); ++i){
        auto seq = sItems[i]->seq();
        double ifps = inputFrameRateCheck.isChecked()  ? inputFrameRateSpin.value()  : seq->frameRate();
        double ofps = outputFrameRateCheck.isChecked() ? outputFrameRateSpin.value() : seq->frameRate();
        
        os << format(_("Applying B-Spline filter to {0}: input frame rate = {1}, output frame rate = {2}"),
                     sItems[i]->name(), ifps, ofps) << endl;
        
        applySplineFilter(*seq, ifps, ofps, ratio);
    }

    ItemList<BodyMotionItem> bItems = items;
    for(int i=0; i < bItems.size(); ++i){
        auto motion = bItems[i]->motion();
        double ifps = inputFrameRateCheck.isChecked()  ? inputFrameRateSpin.value()  : motion->frameRate();
        double ofps = outputFrameRateCheck.isChecked() ? outputFrameRateSpin.value() : motion->frameRate();
        
        os << format(_("Applying B-Spline filter to {0}: input frame rate = {1}, output frame rate = {2}"),
                     bItems[i]->name(), ifps, ofps) << endl;

        applySplineFilter(*motion->jointPosSeq(), ifps, ofps, ratio);
        applySplineFilter(*motion->linkPosSeq(), ifps, ofps, ratio);

        if(auto zmpSeq = getZMPSeq(*motion)){
            applySplineFilter(*zmpSeq, ifps, ofps, ratio);
        }
    }
}

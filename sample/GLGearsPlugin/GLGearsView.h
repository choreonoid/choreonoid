/*
  The base program of this sample is the "3-D gear wheels" sample of gtkglextmm.
  The origiral "3-D gear wheels" is a public domain program written by Brian Paul
  and its conversion to gtkglextmm was wirtten by Naofumi Yasufuku.
*/  

#include <cnoid/View>
#include <cnoid/TimeBar>
#include <QGLWidget>

namespace cnoid {

class GearsScene : public QGLWidget
{
public:
    GearsScene(QWidget* parent = 0);
    bool setTime(double time);

protected:
    virtual void initializeGL();
    virtual void resizeGL(int width, int height);
    virtual void paintGL();
    
private:
    void gear(GLfloat innerRadius, GLfloat outerRadius, GLfloat width, GLint teeth, GLfloat toothDepth);
    
protected:
    GLint gear1;
    GLint gear2;
    GLint gear3;
    GLfloat viewRotX;
    GLfloat viewRotY;
    GLfloat viewRotZ;
    GLfloat angle;
};

    
class GLGearsView : public View
{
public:
    GLGearsView();

protected:
    virtual void onActivated();
    virtual void onDeactivated();
    
private:
    GearsScene* gearsScene;
    cnoid::TimeBar* timeBar;
    Connection timeChangeConnection;
};

}

#include "view.h"

#include "viewformat.h"

#include <QApplication>
#include <QKeyEvent>

#include <iostream>

float step = 3.0f;

using namespace std;

View::View(QWidget *parent) : QGLWidget(ViewFormat(), parent),
    m_window(parent->parentWidget()),
    m_time(), m_timer(),
    m_forward(), m_sideways(), m_vertical(),
    m_lastX(), m_lastY(),
    m_capture(false)
{
    // View needs all mouse move events, not just mouse drag events
    setMouseTracking(true);

    // Hide the cursor since this is a fullscreen app
    QApplication::setOverrideCursor(Qt::ArrowCursor);

    // View needs keyboard focus
    setFocusPolicy(Qt::StrongFocus);

    // The game loop is implemented using a timer
    connect(&m_timer, SIGNAL(timeout()), this, SLOT(tick()));
}

View::~View()
{
    delete m_shader;
}

void View::initializeGL()
{
    glewExperimental = GL_TRUE;
    if(glewInit() != GLEW_OK) {
        std::cerr << "glew initialization failed" << std::endl;
    }
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);


    // alice blue
    glClearColor(240.0f/255.0f, 248.0f/255.0f, 255.0f/255.0f, 1);


//    void SceneviewScene::loadNormalsShader()
//    {
//      std::string vertexSource = ResourceLoader::loadResourceFileToString(":/shaders/normals.vert");
//      std::string geometrySource = ResourceLoader::loadResourceFileToString(":/shaders/normals.gsh");
//      std::string fragmentSource = ResourceLoader::loadResourceFileToString(":/shaders/normals.frag");
//      m_normalsShader = std::make_unique<Shader>(vertexSource, geometrySource, fragmentSource);
//    }

//    void SceneviewScene::loadNormalsArrowShader()
//    {
//      std::string vertexSource = ResourceLoader::loadResourceFileToString(":/shaders/normalsArrow.vert");
//      std::string geometrySource = ResourceLoader::loadResourceFileToString(":/shaders/normalsArrow.gsh");
//      std::string fragmentSource = ResourceLoader::loadResourceFileToString(":/shaders/normalsArrow.frag");
//      m_normalsArrowShader = std::make_unique<Shader>(vertexSource, geometrySource, fragmentSource);
//    }

    m_shader = new Shader(":/shaders/shader.vert", ":/shaders/shader.frag");
    m_normalsArrowShader = new Shader(":/shaders/normals/normalsArrow.vert", ":/shaders/normals/normalsArrow.gsh", ":/shaders/normals/normalsArrow.frag");
    m_normalsShader = new Shader(":/shaders/normals/normals.vert", ":/shaders/normals/normals.gsh", ":/shaders/normals/normals.frag");

    m_sim.init();

    m_camera.setPosition(Eigen::Vector3f(0, 0, 5));
    m_camera.lookAt(Eigen::Vector3f(0, 2, -5), Eigen::Vector3f(0, 2, 0), Eigen::Vector3f(0, 1, 0));
    m_camera.setTarget(Eigen::Vector3f(0, 2, 0));
    m_camera.setPerspective(120, width() / static_cast<float>(height()), 0.1, 50);

    m_time.start();
    m_timer.start(1000 / 60);
}

void View::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f mvp = m_camera.getProjection() * m_camera.getView();
    m_shader->bind();
    m_shader->setUniform("m", model);
    m_shader->setUniform("vp", mvp);
    m_normalsShader->bind();
    m_normalsShader->setUniform("m", model);
    m_normalsShader->setUniform("vp", mvp);
    m_normalsArrowShader->bind();
    m_normalsArrowShader->setUniform("m", model);
    m_normalsArrowShader->setUniform("vp", mvp);
    m_normalsArrowShader->unbind();

    m_sim.draw(m_shader, m_normalsShader, m_normalsArrowShader);

}

void View::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    m_camera.setAspect(static_cast<float>(w) / h);
}

void View::mousePressEvent(QMouseEvent *event) {
    m_capture = true;
    m_lastX = event->x();
    m_lastY = event->y();

    m_deltaX = 0;
    m_deltaY = 0;
}

void View::mouseMoveEvent(QMouseEvent *event)
{
    int deltaX = event->x() - m_lastX;
    int deltaY = event->y() - m_lastY;

    if(m_capture) {
        if(deltaX != 0 || deltaY != 0) {
            m_camera.rotate(-deltaX * 0.01f, deltaY * 0.01f);
        }
    }
    m_lastX = event->x();
    m_lastY = event->y();
    m_deltaX = deltaX;
    m_deltaY = deltaY;
}

void View::mouseReleaseEvent(QMouseEvent *event) {
    m_capture = false;
}

void View::wheelEvent(QWheelEvent *event)
{
    float zoom = 1 - event->delta() * 0.1f / 120;
    m_camera.zoom(zoom);
}

void View::keyPressEvent(QKeyEvent *event)
{
    // Don't remove this -- helper code for key repeat events
    if(event->isAutoRepeat()) {
        keyRepeatEvent(event);
        return;
    }

    // Feel free to remove this
    if (event->key() == Qt::Key_Escape) QApplication::quit();

    if(event->key() == Qt::Key_C)
    {
        m_camera.toggleOrbit();
    }
    else if(event->key() == Qt::Key_W)
    {
        m_forward += step;
    }
    else if(event->key() == Qt::Key_S)
    {
        m_forward -= step;
    }
    else if(event->key() == Qt::Key_A)
    {
        m_sideways -= step;
    }
    else if(event->key() == Qt::Key_D)
    {
        m_sideways += step;
    }
    else if(event->key() == Qt::Key_Q)
    {
        m_vertical -= step;
    }
    else if(event->key() == Qt::Key_E)
    {
        m_vertical += step;
    }
    else if(event->key() == Qt::Key_T)
    {
        m_sim.toggleWire();
    }
    else if(event->key() == Qt::Key_P)
    {
        m_pause = !m_pause;
    }
    else if(event->key() == Qt::Key_Equal) {
        m_sim.tilt_ground(0.01);
    } else if(event->key() == Qt::Key_Minus) {
        m_sim.tilt_ground(-0.01);
    }
 }

void View::keyRepeatEvent(QKeyEvent *)
{
}

void View::keyReleaseEvent(QKeyEvent *event)
{
    // Don't remove this -- helper code for key repeat events
    if(event->isAutoRepeat()) {
        return;
    }
    if(event->key() == Qt::Key_W) {
        m_forward -= step;
    }
    else if(event->key() == Qt::Key_S) {
        m_forward += step;
    }
    else if(event->key() == Qt::Key_A) {
        m_sideways += step;
    }
    else if(event->key() == Qt::Key_D) {
        m_sideways -= step;
    }
    else if(event->key() == Qt::Key_Q) {
        m_vertical += step;
    }
    else if(event->key() == Qt::Key_E) {
        m_vertical -= step;
    }
}

void View::tick()
{
    float seconds = m_time.restart() * 0.001f;

    if (!m_pause) {
        m_sim.update(seconds);
    }

    auto look = m_camera.getLook();
    look.y() = 0;
    look.normalize();
    Eigen::Vector3f perp(-look.z(), 0, look.x());
    Eigen::Vector3f moveVec = m_forward * look + m_sideways * perp + m_vertical * Eigen::Vector3f::UnitY();
    moveVec *= seconds;
    m_camera.move(moveVec);
    // Flag this view for repainting (Qt will call paintGL() soon after)
    update();
}

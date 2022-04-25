QT += core gui opengl

TARGET = smoke-signal
TEMPLATE = app

QMAKE_CXXFLAGS += -mstackrealign
HOME_DIR = $$(HOME)
QMAKE_CXXFLAGS = -I "$${HOME_DIR}/eigen-git-mirror"
INCLUDEPATH += "$${HOME_DIR}/eigen-git-mirror"
DEPENDPATH += "$${HOME_DIR}/eigen-git-mirror"

CONFIG += c++17

unix:!macx {
    LIBS += -lGLU
}
win32 {
    DEFINES += GLEW_STATIC
    LIBS += -lopengl32 -lglu32
}

SOURCES += \
    libs/glew-1.10.0/src/glew.c \
    src/colliders/collider.cpp \
    src/colliders/plane.cpp \
    src/colliders/sphere.cpp \
    src/equations.cpp \
    src/fem/face.cpp \
    src/fem/mesh.cpp \
    src/fem/node.cpp \
    src/fem/tet.cpp \
    src/grid/solver.cpp \
    src/main.cpp \
    src/mainwindow.cpp \
    src/view.cpp \
    src/viewformat.cpp \
    src/graphics/Shader.cpp \
    src/graphics/GraphicsDebug.cpp \
    src/simulation.cpp \
    src/graphics/shape.cpp \
    src/graphics/camera.cpp \
    src/graphics/MeshLoader.cpp \
    src/rendering/rendering.cpp

HEADERS += \
    src/colliders/collider.h \
    src/colliders/plane.h \
    src/colliders/sphere.h \
    src/constants.h \
    src/fem/face.h \
    src/fem/mesh.h \
    src/fem/node.h \
    src/fem/tet.h \
    src/grid/face.h \
    src/grid/grid.h \
    src/grid/solver.h \
    src/grid/voxel.h \
    src/mainwindow.h \
    src/view.h \
    src/viewformat.h \
    src/graphics/Shader.h \
    src/graphics/ShaderAttribLocations.h \
    src/graphics/GraphicsDebug.h \
    src/simulation.h \
    src/graphics/shape.h \
    src/graphics/camera.h \
    src/fem \
    ui_mainwindow.h \
    src/graphics/MeshLoader.h \
    src/rendering/rendering.h

FORMS += src/mainwindow.ui

RESOURCES += \
    res/shaders/shaders.qrc

DISTFILES += \
    res/shaders/shader.vert \
    res/shaders/shader.frag

INCLUDEPATH += src libs glm libs/glew-1.10.0/include libs/Eigen/
DEPENDPATH += src libs glm libs/glew-1.10.0/include libs/Eigen/


#include <X11/Xlib.h>
#include <GL/glx.h>
#include <cstdio>
int main() {
    Display *d = XOpenDisplay(nullptr);
    if (!d) return 1;
    int a[] = {GLX_RGBA, GLX_DOUBLEBUFFER, None};
    XVisualInfo *v = glXChooseVisual(d, DefaultScreen(d), a);
    if (!v) return 2;
    XSetWindowAttributes s{};
    s.colormap = XCreateColormap(d, DefaultRootWindow(d), v->visual, AllocNone);
    Window w = XCreateWindow(d, DefaultRootWindow(d), 0, 0, 1, 1, 0, v->depth,
                            InputOutput, v->visual, CWColormap, &s);
    GLXContext c = glXCreateContext(d, v, nullptr, True);
    if (!c || !glXMakeCurrent(d, w, c)) return 3;
    printf("vendor=%s\nrenderer=%s\nversion=%s\ndirect=%d\n", glGetString(GL_VENDOR),
           glGetString(GL_RENDERER), glGetString(GL_VERSION), glXIsDirect(d, c));
    glXMakeCurrent(d, None, nullptr);
    glXDestroyContext(d, c);
    XDestroyWindow(d, w);
    XCloseDisplay(d);
}

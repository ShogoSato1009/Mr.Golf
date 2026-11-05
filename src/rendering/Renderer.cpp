#include "rendering/Renderer.hpp"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <GL/gl.h>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace golf {
namespace {

Vec3 unit(Vec3 v) { return v/length(v); }
void vertex(Vec3 p) { glVertex3d(p.x, p.y, p.z); }
void normal(Vec3 p) { glNormal3d(p.x, p.y, p.z); }

void sphere(Vec3 centre, double radius) {
    for (int lat = 0; lat < 12; ++lat) {
        glBegin(GL_QUAD_STRIP);
        for (int lon = 0; lon <= 24; ++lon) {
            const double b = 2*pi*lon/24;
            for (int edge = 0; edge < 2; ++edge) {
                const double a = -pi/2+pi*(lat+edge)/12;
                const Vec3 n{std::cos(a)*std::cos(b), std::sin(a), std::cos(a)*std::sin(b)};
                normal(n);
                vertex(centre+radius*n);
            }
        }
        glEnd();
    }
}

LRESULT CALLBACK windowProc(HWND window, UINT message, WPARAM wParam, LPARAM lParam) {
    if (message == WM_CLOSE) { DestroyWindow(window); return 0; }
    if (message == WM_PAINT) {
        PAINTSTRUCT paint{};
        const HDC dc = BeginPaint(window, &paint);
        const auto frame = reinterpret_cast<HDC>(GetWindowLongPtrW(window, GWLP_USERDATA));
        if (frame) {
            RECT area{};
            GetClientRect(window, &area);
            BitBlt(dc, 0, 0, area.right, area.bottom, frame, 0, 0, SRCCOPY);
        }
        EndPaint(window, &paint);
        return 0;
    }
    return DefWindowProcW(window, message, wParam, lParam);
}

std::string number(double value, int precision = 1) {
    std::ostringstream text;
    text << std::fixed << std::setprecision(precision) << value;
    return text.str();
}

} // namespace

struct Renderer::Impl {
    int width, height;
    HDC dc = nullptr;
    HBITMAP bitmap = nullptr;
    HGDIOBJ oldBitmap = nullptr;
    HGLRC context = nullptr;
    HFONT font = nullptr;
    HFONT titleFont = nullptr;
    HWND window = nullptr;
    void* data = nullptr;

    Impl(int w, int h) : width(w), height(h) {}
    ~Impl() {
        if (context) { wglMakeCurrent(nullptr, nullptr); wglDeleteContext(context); }
        if (window && IsWindow(window)) DestroyWindow(window);
        if (font) DeleteObject(font);
        if (titleFont) DeleteObject(titleFont);
        if (oldBitmap) SelectObject(dc, oldBitmap);
        if (bitmap) DeleteObject(bitmap);
        if (dc) DeleteDC(dc);
    }

    void panel(int x, int y, int w, int h, COLORREF color) {
        RECT rect{x, y, x+w, y+h};
        const HBRUSH brush = CreateSolidBrush(color);
        FillRect(dc, &rect, brush);
        DeleteObject(brush);
    }
    void text(int x, int y, const std::string& value, COLORREF color, bool title = false) {
        const auto old = SelectObject(dc, title ? titleFont : font);
        SetTextColor(dc, color);
        SetBkMode(dc, TRANSPARENT);
        TextOutA(dc, x, y, value.c_str(), static_cast<int>(value.size()));
        SelectObject(dc, old);
    }
};

Renderer::Renderer(int width, int height, bool preview) : impl_(std::make_unique<Impl>(width, height)) {
    if (width < 1 || height < 1 || width > 3840 || height > 2160)
        throw std::invalid_argument("Invalid render dimensions");
    auto& p = *impl_;
    p.dc = CreateCompatibleDC(nullptr);
    BITMAPINFO info{};
    info.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    info.bmiHeader.biWidth = width;
    info.bmiHeader.biHeight = height;
    info.bmiHeader.biPlanes = 1;
    info.bmiHeader.biBitCount = 32;
    info.bmiHeader.biCompression = BI_RGB;
    p.bitmap = CreateDIBSection(p.dc, &info, DIB_RGB_COLORS, &p.data, nullptr, 0);
    if (!p.dc || !p.bitmap) throw std::runtime_error("Cannot create offscreen bitmap");
    p.oldBitmap = SelectObject(p.dc, p.bitmap);
    PIXELFORMATDESCRIPTOR format{};
    format.nSize = sizeof(format);
    format.nVersion = 1;
    format.dwFlags = PFD_DRAW_TO_BITMAP | PFD_SUPPORT_OPENGL | PFD_SUPPORT_GDI;
    format.iPixelType = PFD_TYPE_RGBA;
    format.cColorBits = 32;
    format.cDepthBits = 32;
    format.iLayerType = PFD_MAIN_PLANE;
    const int pixelFormat = ChoosePixelFormat(p.dc, &format);
    if (!pixelFormat || !SetPixelFormat(p.dc, pixelFormat, &format))
        throw std::runtime_error("No OpenGL bitmap pixel format");
    p.context = wglCreateContext(p.dc);
    if (!p.context || !wglMakeCurrent(p.dc, p.context))
        throw std::runtime_error("Cannot create OpenGL context");
    p.font = CreateFontW(-18, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
                        OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY,
                        DEFAULT_PITCH, L"Consolas");
    p.titleFont = CreateFontW(-25, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
                             OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY,
                             DEFAULT_PITCH, L"Segoe UI");
    if (preview) {
        const HINSTANCE instance = GetModuleHandleW(nullptr);
        WNDCLASSW cls{};
        cls.lpfnWndProc = windowProc;
        cls.hInstance = instance;
        cls.lpszClassName = L"MrGolfStage2";
        cls.hCursor = LoadCursor(nullptr, IDC_ARROW);
        if (!RegisterClassW(&cls) && GetLastError() != ERROR_CLASS_ALREADY_EXISTS)
            throw std::runtime_error("Cannot register preview window");
        RECT rect{0, 0, width, height};
        constexpr DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
        AdjustWindowRect(&rect, style, FALSE);
        p.window = CreateWindowW(cls.lpszClassName, L"Mr. Golf - debug preview (edit config to replay)",
                                style, CW_USEDEFAULT, CW_USEDEFAULT, rect.right-rect.left,
                                rect.bottom-rect.top, nullptr, nullptr, instance, nullptr);
        if (!p.window) throw std::runtime_error("Cannot create preview window");
        SetWindowLongPtrW(p.window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(p.dc));
        ShowWindow(p.window, SW_SHOW);
    }
}

Renderer::~Renderer() = default;
const unsigned char* Renderer::pixels() const { return static_cast<const unsigned char*>(impl_->data); }
std::size_t Renderer::byteCount() const {
    return static_cast<std::size_t>(impl_->width)*static_cast<std::size_t>(impl_->height)*4;
}

bool Renderer::processEvents() {
    MSG message;
    while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE)) {
        TranslateMessage(&message);
        DispatchMessageW(&message);
    }
    return !impl_->window || IsWindow(impl_->window);
}

void Renderer::draw(const BallState& ball, double radius, const Terrain& terrain,
                    const Camera& camera, const std::vector<Vec3>& trail, const RenderInfo& info) {
    auto& p = *impl_;
    const double aspect = static_cast<double>(p.width)/p.height;
    const double halfY = std::tan(camera.verticalFov()*pi/360);
    const Vec3 eye = camera.eye();
    const Vec3 forward = unit(camera.target()-eye);
    const Vec3 right = unit(cross(forward, {0, 1, 0}));
    const Vec3 up = cross(right, forward);
    const double view[]{right.x, up.x, -forward.x, 0,
                        right.y, up.y, -forward.y, 0,
                        right.z, up.z, -forward.z, 0,
                        -dot(right, eye), -dot(up, eye), dot(forward, eye), 1};
    GdiFlush();
    if (!wglMakeCurrent(p.dc, p.context)) throw std::runtime_error("Lost OpenGL context");
    glViewport(0, 0, p.width, p.height);
    const GLfloat sky[]{0.57f, 0.76f, 0.87f, 1};
    glClearColor(sky[0], sky[1], sky[2], sky[3]);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-halfY*aspect*0.1, halfY*aspect*0.1, -halfY*0.1, halfY*0.1, 0.1, 900);
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd(view);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    const GLfloat light[]{-0.4f, 1.0f, 0.5f, 0};
    const GLfloat ambient[]{0.5f, 0.5f, 0.5f, 1};
    glLightfv(GL_LIGHT0, GL_POSITION, light);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
    glEnable(GL_FOG);
    glFogi(GL_FOG_MODE, GL_LINEAR);
    glFogfv(GL_FOG_COLOR, sky);
    glFogf(GL_FOG_START, 55);
    glFogf(GL_FOG_END, 160);
    // A local mesh follows the camera. Geometry is sampled from the same terrain as physics.
    const double originX = std::floor(camera.target().x/5)*5;
    const double originZ = std::floor(camera.target().z/5)*5;
    for (int ix = -24; ix < 48; ++ix) {
        glBegin(GL_QUAD_STRIP);
        for (int iz = -36; iz <= 36; ++iz) {
            for (int edge = 0; edge < 2; ++edge) {
                const double x = originX+5*(ix+edge), z = originZ+5*iz;
                const auto s = terrain.sample(x, z);
                const bool stripe = std::fmod(std::floor(x/10), 2) == 0;
                const bool fairway = std::abs(z) < 26;
                const double brightness = stripe ? 1 : 0.9;
                glColor3d((fairway ? 0.22 : 0.15)*brightness,
                          (fairway ? 0.49 : 0.36)*brightness, 0.16*brightness);
                normal(s.normal);
                vertex({x, s.height, z});
            }
        }
        glEnd();
    }
    glDisable(GL_LIGHTING);
    glColor3d(0.38, 0.60, 0.30);
    const double markerOriginX = std::floor(originX/10)*10;
    glBegin(GL_LINES);
    for (int ix = -6; ix <= 9; ++ix) {
        const double x = markerOriginX+10*ix;
        for (int iz = -16; iz < 16; ++iz) {
            const double z = originZ+5*iz;
            vertex({x, terrain.sample(x, z).height+0.025, z});
            vertex({x, terrain.sample(x, z+5).height+0.025, z+5});
        }
    }
    glEnd();
    // Ground locator and height stem make the flight/contact boundary visible.
    const auto ground = terrain.sample(ball.position.x, ball.position.z);
    const double locatorRadius = std::max(1.6*radius, 0.018*length(ball.position-eye));
    const double groundLift = std::min(0.02, radius*0.5);
    glColor3d(0.08, 0.16, 0.09);
    glBegin(GL_TRIANGLE_FAN);
    vertex({ball.position.x, ground.height+groundLift, ball.position.z});
    for (int i = 0; i <= 32; ++i) {
        const double a = 2*pi*i/32;
        const double x = ball.position.x+locatorRadius*std::cos(a), z = ball.position.z+locatorRadius*std::sin(a);
        vertex({x, terrain.sample(x, z).height+groundLift, z});
    }
    glEnd();
    glColor3d(0.87, 0.92, 0.70);
    glBegin(GL_LINES);
    vertex({ball.position.x, ground.height+groundLift, ball.position.z});
    vertex(ball.position);
    glEnd();
    glLineWidth(2);
    glColor3d(1.0, 0.74, 0.23);
    glBegin(GL_LINE_STRIP);
    for (const auto& point : trail) vertex(point);
    vertex(ball.position);
    glEnd();
    glLineWidth(1);
    glEnable(GL_LIGHTING);
    glColor3d(0.98, 0.98, 0.96);
    sphere(ball.position, radius);
    glDisable(GL_LIGHTING);
    glDisable(GL_FOG);
    glFinish(); // Complete OpenGL before GDI overlays / CPU reads of the DIB.
    if (glGetError() != GL_NO_ERROR) throw std::runtime_error("OpenGL render failed");

    // A screen-space locator preserves visibility without enlarging the physical sphere.
    const Vec3 relative = ball.position-eye;
    const double depth = dot(relative, forward);
    if (depth > 0.1 && radius*p.height/(2*depth*halfY) < 4.5) {
        const double screenX = 0.5+dot(relative, right)/(2*depth*halfY*aspect);
        const double screenY = 0.5-dot(relative, up)/(2*depth*halfY);
        if (screenX >= 0 && screenX <= 1 && screenY >= 0 && screenY <= 1) {
            const int x = static_cast<int>(screenX*p.width), y = static_cast<int>(screenY*p.height);
            const auto pen = CreatePen(PS_SOLID, 2, RGB(255, 255, 245));
            const auto oldPen = SelectObject(p.dc, pen);
            const auto oldBrush = SelectObject(p.dc, GetStockObject(HOLLOW_BRUSH));
            Ellipse(p.dc, x-7, y-7, x+8, y+8);
            SelectObject(p.dc, oldPen);
            SelectObject(p.dc, oldBrush);
            DeleteObject(pen);
        }
    }
    // Scale the debug overlay from a 1280x720 reference layout.
    const int saved = SaveDC(p.dc);
    SetMapMode(p.dc, MM_ANISOTROPIC);
    SetWindowExtEx(p.dc, 1280, 720, nullptr);
    SetViewportExtEx(p.dc, p.width, p.height, nullptr);
    const COLORREF white = RGB(238, 244, 244), muted = RGB(173, 195, 202), gold = RGB(255, 196, 88);
    p.panel(0, 0, 1280, 98, RGB(20, 38, 46));
    p.text(28, 14, "MR. GOLF  /  PHYSICS LAB", white, true);
    p.text(28, 53, "3D tracking camera  |  fixed-step physics  |  read-only visualization", muted);
    p.text(925, 16, info.phase, gold, true);
    p.text(925, 55, "t "+number(info.time, 2)+" s / "+number(info.duration, 2)+" s", white);
    p.panel(22, 116, 364, 108, RGB(25, 47, 52));
    p.text(36, 127, "WIND  "+number(info.environment.windVelocity.x)+", "+
           number(info.environment.windVelocity.y)+", "+number(info.environment.windVelocity.z)+" m/s", white);
    p.text(36, 155, "PHYSICS  "+number(1/info.physicsDt, 0)+" Hz   VIDEO  "+std::to_string(info.fps)+" fps", muted);
    p.text(36, 183, "10 m markers  |  ball locator ring", muted);
    p.panel(0, 626, 1280, 94, RGB(20, 38, 46));
    p.text(28, 641, "SPEED  "+number(length(ball.velocity), 2)+" m/s", white, true);
    p.text(420, 641, "HEIGHT  "+number(ball.position.y-ground.height, 2)+" m", white, true);
    const auto displacement = ball.position-info.launchPosition;
    p.text(836, 641, "RANGE  "+number(std::hypot(displacement.x, displacement.z), 1)+" m", white, true);
    p.text(28, 682, "x "+number(ball.position.x, 2)+"  y "+number(ball.position.y, 2)+
           "  z "+number(ball.position.z, 2)+" m", muted);
    p.text(620, 682, "SPIN  "+number(length(ball.angularVelocity), 1)+" rad/s", muted);
    if (info.landingPosition) {
        const auto carry = *info.landingPosition-info.launchPosition;
        const auto run = ball.position-*info.landingPosition;
        p.text(836, 682, "CARRY "+number(std::hypot(carry.x, carry.z))+" m  RUN "+
               number(std::hypot(run.x, run.z))+" m", muted);
    }
    p.panel(0, 622, static_cast<int>(1280*std::clamp(info.time/std::max(0.001, info.duration), 0.0, 1.0)), 4, gold);
    RestoreDC(p.dc, saved);
    GdiFlush();
    if (p.window && IsWindow(p.window)) {
        InvalidateRect(p.window, nullptr, FALSE);
        UpdateWindow(p.window);
    }
}

} // namespace golf

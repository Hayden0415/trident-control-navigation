"""Minimal X11 window arrangement for recording native Gazebo and RViz, no custom UI."""
import ctypes as C
import ctypes.util
import time


class Desktop:
    def __init__(self):
        self.x = C.CDLL(ctypes.util.find_library('X11'))
        signatures = {
            'XOpenDisplay': (C.c_void_p, [C.c_char_p]),
            'XDefaultRootWindow': (C.c_ulong, [C.c_void_p]),
            'XDefaultScreen': (C.c_int, [C.c_void_p]),
            'XDisplayWidth': (C.c_int, [C.c_void_p, C.c_int]),
            'XDisplayHeight': (C.c_int, [C.c_void_p, C.c_int]),
            'XInternAtom': (C.c_ulong, [C.c_void_p, C.c_char_p, C.c_int]),
            'XGetWindowProperty': (C.c_int, [C.c_void_p, C.c_ulong, C.c_ulong, C.c_long, C.c_long, C.c_int, C.c_ulong,
                                           C.POINTER(C.c_ulong), C.POINTER(C.c_int), C.POINTER(C.c_ulong), C.POINTER(C.c_ulong), C.POINTER(C.c_void_p)]),
            'XFetchName': (C.c_int, [C.c_void_p, C.c_ulong, C.POINTER(C.c_void_p)]),
            'XMoveResizeWindow': (C.c_int, [C.c_void_p, C.c_ulong, C.c_int, C.c_int, C.c_uint, C.c_uint]),
            'XRaiseWindow': (C.c_int, [C.c_void_p, C.c_ulong]),
            'XSendEvent': (C.c_int, [C.c_void_p, C.c_ulong, C.c_int, C.c_long, C.c_void_p]),
            'XFlush': (C.c_int, [C.c_void_p]), 'XFree': (C.c_int, [C.c_void_p]),
            'XCloseDisplay': (C.c_int, [C.c_void_p]),
        }
        for name, (restype, argtypes) in signatures.items():
            f = getattr(self.x, name); f.restype = restype; f.argtypes = argtypes
        self.d = self.x.XOpenDisplay(None)
        if not self.d: raise RuntimeError('无法连接X11显示；请在本地图形桌面运行')
        self.root = self.x.XDefaultRootWindow(self.d); screen = self.x.XDefaultScreen(self.d)
        self.width = self.x.XDisplayWidth(self.d, screen); self.height = self.x.XDisplayHeight(self.d, screen)

    def atom(self, name): return self.x.XInternAtom(self.d, name.encode(), False)

    def windows(self):
        typ = C.c_ulong(); fmt = C.c_int(); count = C.c_ulong(); after = C.c_ulong(); data = C.c_void_p()
        self.x.XGetWindowProperty(self.d, self.root, self.atom('_NET_CLIENT_LIST'), 0, 10000, False, 0,
                                 C.byref(typ), C.byref(fmt), C.byref(count), C.byref(after), C.byref(data))
        result = {}
        if data.value and fmt.value == 32:
            for w in C.cast(data, C.POINTER(C.c_ulong))[:count.value]:
                name = C.c_void_p(); nt = C.c_ulong(); nf = C.c_int(); nc = C.c_ulong(); na = C.c_ulong()
                self.x.XGetWindowProperty(self.d, w, self.atom('_NET_WM_NAME'), 0, 4096, False, 0,
                                         C.byref(nt), C.byref(nf), C.byref(nc), C.byref(na), C.byref(name))
                if name.value:
                    if nf.value == 8: result[w] = C.string_at(name, nc.value).decode('utf-8', errors='replace')
                    self.x.XFree(name)
                if w not in result:
                    name = C.c_void_p()
                    if self.x.XFetchName(self.d, w, C.byref(name)) and name.value:
                        result[w] = C.string_at(name).decode('utf-8', errors='replace'); self.x.XFree(name)
        if data.value: self.x.XFree(data)
        return result

    def arrange(self, before, timeout=20):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            fresh = {w: title for w, title in self.windows().items() if w not in before}
            gazebo = next((w for w, title in fresh.items() if 'gazebo' in title.lower()), None)
            rviz = next((w for w, title in fresh.items() if 'rviz' in title.lower()), None)
            if gazebo and rviz: break
            time.sleep(.2)
        else: raise RuntimeError('未找到本次启动的Gazebo和RViz窗口；检查环境与图形日志')
        class Client(C.Structure):
            _fields_ = [('type', C.c_int), ('serial', C.c_ulong), ('send_event', C.c_int), ('display', C.c_void_p),
                        ('window', C.c_ulong), ('message_type', C.c_ulong), ('format', C.c_int), ('data', C.c_long*5)]
        for i, window in enumerate((gazebo, rviz)):
            event = C.create_string_buffer(192)
            c = Client.from_buffer(event); c.type = 33; c.display = self.d; c.window = window
            c.message_type = self.atom('_NET_WM_STATE'); c.format = 32
            c.data[:] = [0, self.atom('_NET_WM_STATE_MAXIMIZED_VERT'), self.atom('_NET_WM_STATE_MAXIMIZED_HORZ'), 1, 0]
            self.x.XSendEvent(self.d, self.root, False, (1 << 20) | (1 << 19), C.byref(event))
            self.x.XFlush(self.d); time.sleep(.1)
            self.x.XMoveResizeWindow(self.d, window, i*(self.width//2), 40, self.width//2-12, self.height-100)
            self.x.XRaiseWindow(self.d, window)
        self.x.XFlush(self.d)
        return {'gazebo_window': gazebo, 'rviz_window': rviz, 'desktop_size': [self.width, self.height]}

    def close(self): self.x.XCloseDisplay(self.d)

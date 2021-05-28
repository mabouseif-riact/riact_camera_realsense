import cv2

from .parameter import Parameter, RangeParameter

class Trackbar:
    def __init__(self, parent, param, scale=1):
        self._parent = parent
        self._param = param
        self._scale = int(scale)
        self._callbacks = []

        cv2.createTrackbar(param.name, parent._name, int(param.value*scale), int(param.max*scale), lambda v: self._trackbar_callback(float(v)/scale))
        cv2.setTrackbarMin(param.name, parent._name, int(param.min*scale))

        self._param.add_observer(self._update_value)

    def add_observer(self, fun):
        self._callbacks.append(fun)

    def _trackbar_callback(self, value):
        if self._param.value != value:
            old_value = self._param.value
            self._param._value = value
            for cb in self._callbacks:
                cb(self, self._param, old_value, value)

    def _update_value(self, old, new):
        if old != new:
            cv2.setTrackbarPos(self._param.name, self._parent._name, int(self._param.value*self._scale))


class Window:
    _border = 1
    _handles = {}
    _active_handle = None

    @classmethod
    def _global_keyboard_callback(cls, key):
        if key == 27:
            cls.destroy_all()
        elif cls._active_handle is not None:
            cls._handles[cls._active_handle]._keyboard_callback(key)

    @classmethod
    def is_running(cls):
        return bool([h for h,w in cls._handles.items() if w.is_open()])

    @classmethod
    def update(cls):
        handles = list(cls._handles.keys())
        for h in handles:
            if not cls._handles[h].is_open():
                cls.destroy(h)

    @classmethod
    def destroy(cls, handle):
        cls._handles[handle].close()
        del cls._handles[handle]

    @classmethod
    def destroy_all(cls):
        handles = list(cls._handles.keys())
        for h in handles:
            cls.destroy(h)


    @classmethod
    def spin(cls, wait=1):
        key = cv2.waitKey(wait)
        last_key = key
        while key >= 0:
            last_key = key
            key = cv2.waitKey(wait)
        if last_key >= 0:
            cls._global_keyboard_callback(last_key)
        cls.update()
        return cls.is_running()


    def __init__(self, name):
        if name in Window._handles:
            raise Exception("Window {} already exists!".format(name))
        print("Creating window {}".format(name))
        self._name = name
        cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL)
        cv2.setMouseCallback(self._name, self._mouse_callback)
        self._custom_keyboard_callback = None
        self._custom_mouse_callback = None
        self._trackbars = {}
        Window._handles[name] = self
        Window._active_handle = name

    def close(self):
        print("Closing window {}".format(self._name))
        if self.is_open():
            cv2.destroyWindow(self._name)
        self._name = None

    def is_open(self):
        return self._name is not None and cv2.getWindowProperty(self._name, cv2.WND_PROP_AUTOSIZE) > 0

    def on_show(self, image):
        return image

    def show(self, image):
        if self.is_open() and image is not None:
            image = self.on_show(image)
            color = (50,200,50) if Window._active_handle == self._name else (100,100,100)
            img = cv2.copyMakeBorder(image, Window._border, Window._border, Window._border, Window._border, cv2.BORDER_CONSTANT, value=color)
            cv2.imshow(self._name, img)

    @property
    def name(self):
        return self._name

    @property
    def size(self):
        w,h = cv2.getWindowImageRect(self.name)[-2:]
        return w-2*Window._border, h-2*Window._border

    def title(self, title):
        cv2.setWindowTitle(self._name, title)

    def _keyboard_callback(self, key):
        self._on_key(key)
        if self._custom_keyboard_callback is not None:
            self._custom_keyboard_callback(self, key)

    def _mouse_callback(self, event, x, y, flags, *args):
        if event != cv2.EVENT_MOUSEHWHEEL and event != cv2.EVENT_MOUSEHWHEEL:
            x,y = x - Window._border, y - Window._border
        if event == cv2.EVENT_LBUTTONDOWN:
            Window._active_handle = self._name
        self._on_mouse(event, x, y, flags, *args)
        if self._custom_mouse_callback is not None:
            self._custom_mouse_callback(event, x, y, flags, *args)

    def _on_key(self, key):
        pass

    def _on_mouse(self, event, x, y, flags, *args):
        pass

    def set_keyboard_callback(self, fun):
        self._custom_keyboard_callback = fun

    def set_mouse_callback(self, fun):
        self._custom_mouse_callback = fun


    def add_control_parameter(self, param, scale=1, on_change=None):
        trackbar = Trackbar(self, param, scale)
        if on_change is not None:
            trackbar.add_observer(on_change)
        self._trackbars[param.name] = trackbar
        return param


if __name__ == '__main__':

    img = cv2.imread("./data/test.jpg", cv2.IMREAD_UNCHANGED)
    win = Window("Test")
    # win.set_mouse_callback(lambda event, x, y, flags, args: win.overlay("{} at {},{}".format(event, x, y)))

    p1 = RangeParameter("Param1", 10, 0, 100)
    p2 = RangeParameter("Param2", 0, -50, 50)
    p3 = RangeParameter("Param3", 0, 0, 1)
    win.add_control_parameter(p1)
    win.add_control_parameter(p2)
    win.add_control_parameter(p3, 100)

    def cb(key):
        if key == 32:
            p1.value = p1.value + 1
            p2.value += 2
        elif key == ord('1'):
            print(p1.name, p1.value)
        elif key == ord('2'):
            print(p2.name, p2.value)
        elif key == ord('3'):
            print(p3.name, p3.value)
    win.set_keyboard_callback(cb)


    win2 = Window("Other")

    p4 = RangeParameter("Param4", 10, 0, 100)
    p5 = RangeParameter("Param5", 0, -50, 50)
    win2.add_control_parameter(p4)
    win2.add_control_parameter(p5,10)

    def cb2(key):
        if key == 32:
            p4.value = p4.value + 1
            p5.value += 20
        elif key == ord('1'):
            print(p4.name, p4.value)
        elif key == ord('2'):
            print(p5.name, p5.value)
    win2.set_keyboard_callback(cb2)



    while Window.spin():
        win.show(img)
        win2.show(img)

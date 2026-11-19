"""収録中のMjDataを右側のGLFWウィンドウへ描画する。"""
import glfw
import mujoco


class SceneWindow:
    def __init__(self, model, x, width, height):
        if not glfw.init():
            raise RuntimeError("GLFWを初期化できません")
        self.window = glfw.create_window(width, height, "MuJoCo | ToPoDualArm", None, None)
        if not self.window:
            raise RuntimeError("MuJoCo表示用ウィンドウを作成できません")
        glfw.set_window_pos(self.window, x, 55)
        previous = glfw.get_current_context()
        glfw.make_context_current(self.window)
        try:
            glfw.swap_interval(0)
            self.render_size = None
            self.scene = mujoco.MjvScene(model, maxgeom=5000)
            self.context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_100)
        finally:
            glfw.make_context_current(previous)
        self.options = mujoco.MjvOption()
        self.camera = mujoco.MjvCamera()
        self.camera.lookat[:] = [.12, 0, .12]
        self.camera.distance = 1.35
        self.camera.azimuth = 225
        self.camera.elevation = -22
        self.mouse = None

        def drag(window, x, y):
            if self.mouse and glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT):
                dx, dy = x-self.mouse[0], y-self.mouse[1]
                self.camera.azimuth -= dx*.4
                self.camera.elevation = max(-89, min(89, self.camera.elevation+dy*.4))
            self.mouse = (x,y)

        def zoom(window, dx, dy):
            self.camera.distance = max(.2,min(5,self.camera.distance*.9**dy))

        glfw.set_cursor_pos_callback(self.window, drag)
        glfw.set_scroll_callback(self.window, zoom)

    def draw(self, model, data):
        if not self.window:
            return
        glfw.poll_events()
        if glfw.window_should_close(self.window):
            self.close()
            return
        previous = glfw.get_current_context()
        glfw.make_context_current(self.window)
        try:
            size = glfw.get_framebuffer_size(self.window)
            if min(size) > 0:
                mujoco.mjv_updateScene(model,data,self.options,None,self.camera,
                                       mujoco.mjtCatBit.mjCAT_ALL,self.scene)
                # 全体表示のみ長辺480pxで描画し、GPU上でウィンドウに拡大する。
                scale = min(1., 480 / max(size))
                low = tuple(max(1, round(v * scale)) for v in size)
                if self.render_size != low:
                    mujoco.mjr_resizeOffscreen(*low, self.context)
                    self.render_size = low
                    print(f"全体ビュー描画 {low} → {size}", flush=True)
                self.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
                self.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
                viewport = mujoco.MjrRect(0, 0, *low)
                mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.context)
                mujoco.mjr_render(viewport, self.scene, self.context)
                mujoco.mjr_blitBuffer(viewport, mujoco.MjrRect(0, 0, *size), 1, 0, self.context)
                mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_WINDOW, self.context)
                glfw.swap_buffers(self.window)
        finally:
            glfw.make_context_current(previous)

    def close(self):
        if self.window:
            previous = glfw.get_current_context()
            glfw.make_context_current(self.window)
            self.context.free()
            glfw.make_context_current(previous)
            glfw.destroy_window(self.window)
            self.window = None

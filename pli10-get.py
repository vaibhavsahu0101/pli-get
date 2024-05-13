    def select_and_init_camera(self,model_name):
        '''Select and initialize a camera based on the model name.'''
        camera = None
        try:
            devices = (pylon.TlFactory.GetInstance()).EnumerateDevices()
            if len(devices) == 0:
                print("No camera found: returning empty camera.")
                return camera
            
            for device in devices:
                if model_name in device.GetModelName():
                    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(device))
                    print("Got camera:", camera.GetDeviceInfo().GetModelName())
                    break
        except:
            print("Camera not found: returning empty camera.")
        return camera
    def set_light_source_preset(self, model_name, light_source_preset='Off'):
        '''Set the light source preset of the camera.'''

        if self.camera is None:
            print(f"Empty camera: {light_source_preset}")
            return

        self.camera.Open()
        if model_name == "acA5472-17uc":
            self.camera.LightSourcePreset.Value = light_source_preset
        
        elif model_name == "a2A4504-18ucBAS":
            self.camera.BslLightSourcePreset.Value = light_source_preset
        if args.model_name is None:
            raise ValueError("model_name is required")
        model_name = args.model_name
        color_mode = args.color_mode
        pli.__init__(model_name)

        pli.set_light_source_preset(light_source_preset)

    parser.add_argument('--model_name', type=str, help='camera model name') 

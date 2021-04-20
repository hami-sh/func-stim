import json, os
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

NODES = [
    'thumb_01', 'thumb_02', 'thumb_03', 'index_01', 'index_02', 'index_03', 'middle_01', 'middle_02', 'middle_03', 'ring_01', 'ring_02', 'ring_03', 'pinky_01', 'pinky_02', 'pinky_03'
]

class Recording:
    def __init__(self, filename: str):
        self.filename = filename
        self.fp = open(self.filename, "r")
        self.fp = json.load(self.fp)
        # print(json.dumps(self.fp, indent=2))

        self.bone_keyframes = dict()
        self.max_keytime = None
        self.delta_keytime = None
        self.hand_side = None
        self.load_keyframes()
        # print(json.dumps(self.bone_keyframes, indent=1))
        # print(self.delta_keytime)
    
    def load_keyframes(self):
        bone_animations = self.fp['animations'][0]['bones']
        for bone_object in bone_animations:
            bone_id = bone_object['boneId']
            if "RightHand" in bone_id or "LeftHand" in bone_id:
                bone_id = bone_id.split("_")[0]
                self.hand_side = bone_id

            keyframes = bone_object['keyframes']
            self.bone_keyframes[bone_id] = keyframes
        
        # calc max keytime
        for keyframe in self.bone_keyframes[self.hand_side]:
            if self.max_keytime is None or keyframe["keytime"] > self.max_keytime:
                self.max_keytime = keyframe["keytime"]
        
        # calc delta keytime
        self.delta_keytime = self.bone_keyframes[self.hand_side][1]['keytime'] - self.bone_keyframes[self.hand_side][0]['keytime']
        
        # add any missing bones with nil rotation change
        for expected_bone in NODES:
            if self.hand_side == "RightHand":
                expected_bone = expected_bone + "_r"
            else:
                expected_bone = expected_bone + "_l"
                
            if expected_bone not in self.bone_keyframes.keys():
                print(expected_bone)             
                self.bone_keyframes[expected_bone] = list()
                
                for keytime in np.arange(0, self.max_keytime+self.delta_keytime, self.delta_keytime):
                    value = dict()
                    value['keytime'] = keytime
                    value['rotation'] = [0.0, 0.0, 0.0, 0.0]
                    self.bone_keyframes[expected_bone].append(value)
            

    
    def print_settling_graph(self):
        frames = list()

        values = dict()
        i = 0
        for bone in self.bone_keyframes.keys():
            values[bone] = dict()
            for axis in ['x', 'z', 'y', 'w']:

                if axis not in values[bone].keys():
                    values[bone][axis] = dict()
                    values[bone][axis]['keytimes'] = list()
                    values[bone][axis]['values'] = list()

                for keyframe in self.bone_keyframes[bone]:
                    values[bone][axis]['keytimes'].append(keyframe['keytime'])
                    values[bone][axis]['values'].append(keyframe['rotation'][i])


                i += 1
                if i == 4:
                    i = 0
        print(json.dumps(values, indent=2))
        
        # for bone in values:
            # df = pd.DataFrame()
        
        # plt.plot(values['RightHand']['x']['keytimes'], values['RightHand']['x']['values'], 'o', color='black')

        NODES.insert(0, self.hand_side)
        fig, axs = plt.subplots(16,figsize=(20, 30), constrained_layout=True)
        i = 0
        for bone in NODES:
            if bone != self.hand_side:
                if self.hand_side == "RightHand":
                    bone = bone + "_r"
                else:
                    bone = bone + "_l"
            print(i)

            axs[i].plot(values[bone]['x']['keytimes'], values[bone]['x']['values'], linestyle='-', color='black')
            axs[i].plot(values[bone]['y']['keytimes'], values[bone]['y']['values'], linestyle='-', color='red')
            axs[i].plot(values[bone]['z']['keytimes'], values[bone]['z']['values'], linestyle='-', color='blue')
            axs[i].plot(values[bone]['w']['keytimes'], values[bone]['w']['values'], linestyle='-', color='green')
            # axs[i].title = bone 
            axs[i].set_title(bone)

            i += 1

        fig.show()
        fig.savefig("images/" + self.filename.split(".g3dj")[0])
        


        # df=pd.DataFrame({
        # 'x_values': range(0,self.max_keytime), 
        # 'RightHand': np.random.randn(10), 
        # 'thumb_01_r': np.random.randn(10)+range(1,11), 
        # 'thumb_02_r': np.random.randn(10)+range(11,21) 
        # 'thumb_03_r': np.random.randn(10)+range(11,21) 
        # 'index_01_r': np.random.randn(10)+range(11,21) 
        # 'index_02_r': np.random.randn(10)+range(11,21) 
        # 'index_03_r': np.random.randn(10)+range(11,21) 
        # 'middle_01_r': np.random.randn(10)+range(11,21) 
        # 'middle_02_r': np.random.randn(10)+range(11,21) 
        # 'middle_03_r': np.random.randn(10)+range(11,21) 
        # 'ring_01_r': np.random.randn(10)+range(11,21) 
        # 'ring_02_r': np.random.randn(10)+range(11,21) 
        # 'ring_03_r': np.random.randn(10)+range(11,21) 
        # 'pinky_01_r': np.random.randn(10)+range(11,21) 
        # 'pinky_02_r': np.random.randn(10)+range(11,21) 
        # 'pinky_03_r': np.random.randn(10)+range(11,21) 
        # })


def main():
    # print(os.listdir())
    r = Recording("thumb_to_pinkie_flexion.g3dj")
    r.print_settling_graph()

if __name__ == "__main__":
    main()
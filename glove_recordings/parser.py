import json, os
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

NODES = [
    'thumb_01', 'thumb_02', 'thumb_03', 'index_01', 'index_02', 'index_03', 'middle_01', 'middle_02', 'middle_03',
    'ring_01', 'ring_02', 'ring_03', 'pinky_01', 'pinky_02', 'pinky_03'
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

        self.values = None
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
        self.delta_keytime = self.bone_keyframes[self.hand_side][1]['keytime'] - self.bone_keyframes[self.hand_side][0][
            'keytime']

        # add any missing bones with nil rotation change
        for expected_bone in NODES:
            if self.hand_side == "RightHand":
                expected_bone = expected_bone + "_r"
            else:
                expected_bone = expected_bone + "_l"

            if expected_bone not in self.bone_keyframes.keys():
                print(expected_bone)
                self.bone_keyframes[expected_bone] = list()

                for keytime in np.arange(0, self.max_keytime + self.delta_keytime, self.delta_keytime):
                    value = dict()
                    value['keytime'] = keytime
                    value['rotation'] = [0.0, 0.0, 0.0, 0.0]
                    self.bone_keyframes[expected_bone].append(value)

    def collate_values(self):
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
        self.values = values



    def print_settling_graph(self):
        NODES.insert(0, self.hand_side)
        fig, axs = plt.subplots(16, figsize=(20, 30), constrained_layout=True)
        i = 0
        for bone in NODES:
            if bone != self.hand_side:
                if self.hand_side == "RightHand":
                    bone = bone + "_r"
                else:
                    bone = bone + "_l"
            print(i)

            axs[i].plot(self.values[bone]['x']['keytimes'], self.values[bone]['x']['values'], linestyle='-', color='black')
            axs[i].plot(self.values[bone]['y']['keytimes'], self.values[bone]['y']['values'], linestyle='-', color='red')
            axs[i].plot(self.values[bone]['z']['keytimes'], self.values[bone]['z']['values'], linestyle='-', color='blue')
            axs[i].plot(self.values[bone]['w']['keytimes'], self.values[bone]['w']['values'], linestyle='-', color='green')
            axs[i].set_title(bone)

            i += 1

        fig.show()
        # plt.plot(values['RightHand']['x']['keytimes'], values['RightHand']['x']['values'], 'o', color='black')
        fig.savefig("images/" + self.filename.split(".g3dj")[0])

    def determine_final_positions(self):
        # reverse traverse (final position should be at the end)
        self.total_settled = list()
        self.bones_settled = dict()

        for bone in NODES:
            if bone not in self.bones_settled.keys():
                self.bones_settled[bone] = list()

            for i in range(0, len(self.values[bone]['x']['keytimes'])):  # [::-1]
                previous_keytimes = list()
                lookback_time = 2000  # 2 seconds look back time
                # j = i
                # while j >= 0 and self.values[bone]['x']['keytimes'][i] - self.values[bone]['x']['keytimes'][j] < lookback_time:

                # get the values in the window
                for j in range(0, i):
                    time_diff = self.values[bone]['x']['keytimes'][i] - self.values[bone]['x']['keytimes'][j]
                    if 0 < time_diff <= lookback_time:
                        """ we are in the settling window """
                        previous_keytimes.append(self.values[bone]['x']['values'][i])

                # check the values in the window
                for 



def main():
    # print(os.listdir())
    r = Recording("thumb_to_pinkie_flexion.g3dj")
    # r.print_settling_graph()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car and train a model for it. 

Usage:
    manage.py (drive) [--model=<model>] [--js]
    manage.py (train) [--tub=<tub1,tub2,..tubn>] (--model=<model>)
    manage.py (calibrate)
    manage.py (check) [--tub=<tub1,tub2,..tubn>] [--fix]
    manage.py (analyze) [--tub=<tub1,tub2,..tubn>] [--op=histogram] [--rec=<record>]
    manage.py (plot_predictions) [--tub=<tub1,tub2,..tubn>] (--model=<model>)
    manage.py (custom_train) [--tub=<tub1,tub2,..tubn>] (--model=<model>)

    Options:
    -h --help     Show this screen.
    --js          Use physical joystick.
    --fix         Remove records which cause problems.

"""


import os
from docopt import docopt
import donkeycar as dk
import pdb


def drive(cfg, model_path=None, use_joystick=False):
    import sys
    
    #Initialized car
    V = dk.vehicle.Vehicle()
    cam = dk.parts.PiCamera(resolution=cfg.CAMERA_RESOLUTION)
    V.add(cam, outputs=['cam/image_array'], threaded=True)
    

    if use_joystick or cfg.USE_JOYSTICK_AS_DEFAULT:
        #modify max_throttle closer to 1.0 to have more power
        #modify steering_scale lower than 1.0 to have less responsive steering
        ctr = dk.parts.JoystickController(max_throttle=cfg.JOYSTICK_MAX_THROTTLE,
                                    steering_scale=cfg.JOYSTICK_STEERING_SCALE,
                                    auto_record_on_throttle=cfg.AUTO_RECORD_ON_THROTTLE)
    else:        
        #This web controller will create a web server that is capable
        #of managing steering, throttle, and modes, and more.
        ctr = dk.parts.LocalWebController()

    V.add(ctr, 
          inputs=['cam/image_array'],
          outputs=['user/angle', 'user/throttle', 'user/mode', 'recording', 'brake'],
          threaded=True)

    #LED indicator for recording
    recording_indicator_part = dk.parts.GPIOPinOutput(cfg.RECORDING_LED)
    V.add(recording_indicator_part, inputs=['recording'])
        
    #See if we should even run the pilot module. 
    #This is only needed because the part run_contion only accepts boolean
    def pilot_condition(mode):
        if mode == 'user':
            return False
        else:
            return True
        
    pilot_condition_part = dk.parts.Lambda(pilot_condition)
    V.add(pilot_condition_part, inputs=['user/mode'], outputs=['run_pilot'])
    
    #Run the pilot if the mode is not user.
    kl = dk.parts.KerasCategorical()
    if model_path:
        kl.load(model_path)
    
    V.add(kl, inputs=['cam/image_array'], 
          outputs=['pilot/angle', 'pilot/throttle'],
          run_condition='run_pilot')
    
    
    #Choose what inputs should change the car.
    def drive_mode(mode, 
                   user_angle, user_throttle,
                   pilot_angle, pilot_throttle):
        if mode == 'user' or model_path is None:
            return user_angle, user_throttle
        
        elif mode == 'local_angle':
            return pilot_angle, user_throttle
        
        else:
            return pilot_angle, pilot_throttle
        
    drive_mode_part = dk.parts.Lambda(drive_mode)
    V.add(drive_mode_part, 
          inputs=['user/mode', 'user/angle', 'user/throttle',
                  'pilot/angle', 'pilot/throttle'], 
          outputs=['angle', 'target_throttle'])
    
    odometer = dk.parts.RotaryEncoder(mm_per_tick=cfg.ROTARY_ENCODER_MM_PER_TICK, pin=cfg.ROTARY_ENCODER_PIN)
    V.add(odometer, outputs=['odometer/meters', 'odometer/meters_per_second'], threaded=True)

    #Transform the velocity measured by the odometer into -1/1 scale
    #so existing controls and modelsbased on -1/1 range can still be used
    def velocity_to_throttle(current_velocity, target_throttle):
      max_velocity = cfg.MAX_VELOCITY

      if target_throttle < 0:
        direction = -1
      else:
        direction = 1

      measured_throttle = (current_velocity/max_velocity)*direction
      
      return measured_throttle

    velocity_to_throttle_part = dk.parts.Lambda(velocity_to_throttle)
    V.add(velocity_to_throttle_part,
          inputs=['odometer/meters_per_second', 'target_throttle'],
          outputs=['measured_throttle'])

    pid = dk.parts.PIDController(p=cfg.THROTTLE_PID_P, d=cfg.THROTTLE_PID_D, i=cfg.THROTTLE_PID_I)
    V.add(pid, 
          inputs=['target_throttle', 'measured_throttle'],
          outputs=['pid/output'])

    #Calculate the new throttle value using output from PID
    #and clamp it to the -1/1 range
    def throttle_with_pid(target_throttle, pid_output):
      pid_throttle = target_throttle + pid_output
      
      if pid_throttle > 1.0:
        pid_throttle = 1.0
      elif pid_throttle < -1.0:
        pid_throttle = -1.0

      return pid_throttle

    throttle_with_pid_part = dk.parts.Lambda(throttle_with_pid)
    V.add(throttle_with_pid_part,
          inputs=['target_throttle','pid/output'],
          outputs=['pid_throttle'])

    steering_controller = dk.parts.PCA9685(cfg.STEERING_CHANNEL)
    steering = dk.parts.PWMSteering(controller=steering_controller,
                                    left_pulse=cfg.STEERING_LEFT_PWM, 
                                    right_pulse=cfg.STEERING_RIGHT_PWM)
    
    throttle_controller = dk.parts.PCA9685(0)
    throttle = dk.parts.PWMThrottle(controller=throttle_controller,
                                    max_pulse=cfg.THROTTLE_FORWARD_PWM, 
                                    zero_pulse=cfg.THROTTLE_STOPPED_PWM, 
                                    min_pulse=cfg.THROTTLE_REVERSE_PWM)

    V.add(steering, inputs=['angle'])

    def throttle_with_brake(throttle, brake):
      if brake:
        print("Brake is on.")
        return 0
      else:
        return throttle

    throttle_with_brake_part = dk.parts.Lambda(throttle_with_brake)
    V.add(throttle_with_brake_part,
            inputs=['pid_throttle', 'brake'],
            outputs=['throttle'])

    #Pass the final throttle value into the controller instead of the
    #raw throttle value from the user or pilot
    V.add(throttle, inputs=['throttle'])
    
    def console_output(mode, user_angle, user_throttle, pilot_angle, pilot_throttle, 
                       distance, velocity, 
                       target_throttle, measured_throttle, pid_throttle):
        if (cfg.DEBUG):
            string = "Mode: " + mode

            if mode == 'user':1
                string += "\nAngle: " + str(round(user_angle, 2))
                string += " | Throttle: " + str(round(user_throttle, 2))

            elif mode == 'local_angle':
                string += "\nAngle: " + str(round(pilot_angle, 2))
                string += " | Throttle: " + str(round(user_throttle, 2))
            
            else:
                string += "\nAngle: " + str(round(pilot_angle, 2))
                string += " | Throttle: " + str(round(pilot_throttle, 2))

            string += "\nDistance: " + str(round(distance, 4)) + " m"
            string += " | Velocity: " + str(round(velocity, 4)) + " m/s"

            string += "\nTarget T: " + str(round(target_throttle, 2))
            string += " | Measured T: " + str(round(measured_throttle, 2))
            string += " | PID T: " + str(round(pid_throttle, 4))

            sys.stdout.write(string)
            sys.stdout.flush()

    console_output_part = dk.parts.Lambda(console_output)
    V.add(console_output_part,
            inputs=['user/mode', 'user/angle', 'user/throttle',
                    'pilot/angle', 'pilot/throttle', 
                    'odometer/meters', 'odometer/meters_per_second',
                    'target_throttle', 'measured_throttle', 'pid_throttle'])

    #add tub to save data
    inputs=['cam/image_array',
            'user/angle', 'user/throttle', 
            #'pilot/angle', 'pilot/throttle', 
            'user/mode',
            'odometer/meters', 'odometer/meters_per_second',
            'target_throttle', 'measured_throttle', 'pid_throttle']
    types=['image_array',
           'float', 'float',  
           #'float', 'float', 
           'str', 
           'float', 'float', 
           'float', 'float', 'float']
    
    th = dk.parts.TubHandler(path=cfg.DATA_PATH)
    tub = th.new_tub_writer(inputs=inputs, types=types)
    V.add(tub, inputs=inputs, run_condition='recording')
    
    #run the vehicle for 20 seconds
    V.start(rate_hz=cfg.DRIVE_LOOP_HZ, 
            max_loop_count=None)

    print("You can now go to <your pi ip address>:8887 to drive your car.")


def expand_path_masks(paths):
    '''
    take a list of paths and expand any wildcards
    returns a new list of paths fully expanded
    '''
    import glob
    expanded_paths = []
    for path in paths:
        if '*' in path or '?' in path:
            mask_paths = glob.glob(path)
            expanded_paths += mask_paths
        else:
            expanded_paths.append(path)

    return expanded_paths


def gather_tubs(cfg, tub_names):
    
    if tub_names:
        tub_paths = [os.path.expanduser(n) for n in tub_names.split(',')]
        tub_paths = expand_path_masks(tub_paths)
    else:
        tub_paths = [os.path.join(cfg.DATA_PATH, n) for n in os.listdir(cfg.DATA_PATH)]

    tubs = [dk.parts.Tub(p) for p in tub_paths]
    return tubs


def train(cfg, tub_names, model_name):
    '''
    use the specified data in tub_names to train an artifical neural network
    saves the output trained model as model_name
    '''
    X_keys = ['cam/image_array']
    y_keys = ['user/angle', 'user/throttle']
    
    def rt(record):
        record['user/angle'] = dk.utils.linear_bin(record['user/angle'])
        return record

    kl = dk.parts.KerasCategorical()
    
    tubs = gather_tubs(cfg, tub_names)

    import itertools

    gens = [tub.train_val_gen(X_keys, y_keys, record_transform=rt, batch_size=cfg.BATCH_SIZE, train_split=cfg.TRAIN_TEST_SPLIT) for tub in tubs]

    # Training data generator is the one that keeps cycling through training data generator of all tubs chained together
    # The same for validation generator
    train_gens = itertools.cycle(itertools.chain(*[gen[0] for gen in gens]))
    val_gens = itertools.cycle(itertools.chain(*[gen[1] for gen in gens]))

    model_path = os.path.expanduser(model_name)

    total_records = sum([t.get_num_records() for t in tubs])
    total_train = int(total_records * cfg.TRAIN_TEST_SPLIT)
    total_val = total_records - total_train
    print('train: %d, validation: %d' %(total_train, total_val))
    steps_per_epoch = total_train // cfg.BATCH_SIZE
    print('steps_per_epoch', steps_per_epoch)

    kl.train(train_gens, 
        val_gens, 
        saved_model_path=model_path,
        steps=steps_per_epoch,
        train_split=cfg.TRAIN_TEST_SPLIT)


def calibrate():
    channel = int(input('Enter the channel your actuator uses (0-15).'))
    c = dk.parts.PCA9685(channel)
    
    for i in range(10):
        pmw = int(input('Enter a PWM setting to test(100-600)'))
        c.run(pmw)

def check(cfg, tub_names, fix=False):
    '''
    Check for any problems. Looks at tubs and find problems in any records or images that won't open.
    If fix is True, then delete images and records that cause problems.
    '''
    tubs = gather_tubs(cfg, tub_names)

    for tub in tubs:
        tub.check(fix=fix)

def anaylze(cfg, tub_names, op, record):
    '''
    look at the tub data and produce some analysis, for example:
    manage.py analyze --tub=<tub1,tub2,..tubn> --op=histogram --rec="user/angle"
    '''
    import json
    import glob

    tubs = gather_tubs(cfg, tub_names)

    if op == 'histogram':
        import matplotlib.pyplot as plt
        samples = []
        for tub in tubs:
            record_paths = glob.glob(os.path.join(tub.path, 'record_*.json'))
            
            for record_path in record_paths:
                with open(record_path, 'r') as fp:
                    json_data = json.load(fp)
                sample = json_data[record]
                samples.append(float(sample))

        plt.hist(samples, 50)
        plt.xlabel(record)
        plt.show()

def plot_predictions(cfg, tub_names, model_name):
    '''
    Plot model predictions for angle and throttle against data from tubs.

    '''
    import matplotlib.pyplot as plt
    import pandas as pd
    from PIL import Image
    import json
    import glob
    import numpy as np

    tubs = gather_tubs(cfg, tub_names)
    
    model_path = os.path.expanduser(model_name)
    model = dk.parts.KerasCategorical()
    model.load(model_path)

    user_angles = []
    user_angles_binned = []
    user_throttles = []
    pilot_angles = []
    pilot_throttles = []

    for tub in tubs:
        record_paths = glob.glob(os.path.join(tub.path, 'record_*.json'))

        for record_path in record_paths:
            with open(record_path, 'r') as fp:
                json_data = json.load(fp)
            
            image_filename = json_data["cam/image_array"]
            image_path = os.path.join(tub.path, image_filename)
            img = Image.open(image_path)
            img = np.array(img)

            binned_angle = dk.utils.linear_bin(json_data['user/angle'])
            user_angle_binned = dk.utils.linear_unbin(binned_angle)

            user_angle = float(json_data["user/angle"])
            user_throttle = float(json_data["user/throttle"])
            pilot_angle, pilot_throttle = model.run(img)

            user_angles.append(user_angle)
            user_angles_binned.append(user_angle_binned)
            user_throttles.append(user_throttle)
            pilot_angles.append(pilot_angle)
            pilot_throttles.append(pilot_throttle)

    angles_df = pd.DataFrame({'user_angle': user_angles, 'pilot_angle': pilot_angles})
    throttles_df = pd.DataFrame({'user_throttle': user_throttles, 'pilot_throttle': pilot_throttles})

    fig = plt.figure()

    title = "Model Predictions\nTubs: " + tub_names + "\nModel: " + model_name
    fig.suptitle(title)

    ax1 = fig.add_subplot(211)
    ax2 = fig.add_subplot(212)

    angles_df.plot(ax=ax1)
    throttles_df.plot(ax=ax2)

    ax1.legend(loc=4)
    ax2.legend(loc=4)

    plt.show()

def custom_train(cfg, tub_names, model_name):
    '''
    use the specified data in tub_names to train an artifical neural network
    saves the output trained model as model_name
    '''
    import sklearn
    from sklearn.model_selection import train_test_split
    from sklearn.utils import shuffle
    import random
    from PIL import Image
    import numpy as np
    import glob
    import json

    images = []
    angles = []
    throttles = []

    tubs = gather_tubs(cfg, tub_names)

    for tub in tubs:
        record_paths = glob.glob(os.path.join(tub.path, 'record_*.json'))
        for record_path in record_paths:

            with open(record_path, 'r') as fp:
                json_data = json.load(fp)

            user_angle = dk.utils.linear_bin(json_data['user/angle'])
            user_throttle = float(json_data["user/throttle"])
            image_filename = json_data["cam/image_array"]
            image_path = os.path.join(tub.path, image_filename)
            
            if (user_angle[7] != 1.0):
                #if the categorical angle is not in the 0 bucket, always include it
                images.append(image_path)
                angles.append(user_angle)
                throttles.append(user_throttle)
            elif (random.randint(0, 9) < 10):
                #Drop a percentage of records where categorical angle is in the 0 bucket
                #increase the number in the conditional above to include more records
                #(< 2 = 20% of 0 angle records included, < 3 = 30% of 0 angle records included, etc.)
                images.append(image_path)
                angles.append(user_angle)
                throttles.append(user_throttle)

    #shuffle and split the data
    train_images, val_images, train_angles, val_angles, train_throttles, val_throttles = train_test_split(images, angles, throttles, shuffle=True, test_size=(1 - cfg.TRAIN_TEST_SPLIT))

    def generator(images, angles, throttles, batch_size=cfg.BATCH_SIZE):
        num_records = len(images)

        while True:
            #shuffle again for good measure
            shuffle(images, angles, throttles)

            for offset in range(0, num_records, batch_size):
                batch_images = images[offset:offset+batch_size]
                batch_angles = angles[offset:offset+batch_size]
                batch_throttles = throttles[offset:offset+batch_size]

                augmented_images = []
                augmented_angles = []
                augmented_throttles = []

                for image_path, angle, throttle in zip(batch_images, batch_angles, batch_throttles):
                    image = Image.open(image_path)
                    image = np.array(image)
                    augmented_images.append(image)
                    augmented_angles.append(angle)
                    augmented_throttles.append(throttle)

                    # if (angle[7] != 1.0):
                    #     #augment the data set with flipped versions of the nonzero angle records
                    #     augmented_images.append(np.fliplr(image))
                    #     augmented_angles.append(np.flip(angle, axis=0))
                    #     augmented_throttles.append(throttle)

                augmented_images = np.array(augmented_images)
                augmented_angles =  np.array(augmented_angles)
                augmented_throttles = np.array(augmented_throttles)

                shuffle(augmented_images, augmented_angles, augmented_throttles)

                X = [augmented_images]
                y = [augmented_angles, augmented_throttles]

                yield X, y

    train_gen = generator(train_images, train_angles, train_throttles)
    val_gen = generator(val_images, val_angles, val_throttles)

    kl = dk.parts.KerasCategorical()
    
    tubs = gather_tubs(cfg, tub_names)
    model_path = os.path.expanduser(model_name)

    total_records = len(images)
    total_train = len(train_images)
    total_val = len(val_images)

    print('train: %d, validation: %d' %(total_train, total_val))
    steps_per_epoch = total_train // cfg.BATCH_SIZE
    print('steps_per_epoch', steps_per_epoch)

    kl.train(train_gen, 
        val_gen, 
        saved_model_path=model_path,
        steps=steps_per_epoch,
        train_split=cfg.TRAIN_TEST_SPLIT)


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()
    
    if args['drive']:
        drive(cfg, model_path = args['--model'], use_joystick=args['--js'])
    
    elif args['calibrate']:
        calibrate()
    
    elif args['train']:
        tub = args['--tub']
        model = args['--model']
        train(cfg, tub, model)

    elif args['check']:
        tub = args['--tub']
        fix = args['--fix']
        check(cfg, tub, fix)

    elif args['analyze']:
        tub = args['--tub']
        op = args['--op']
        rec = args['--rec']
        anaylze(cfg, tub, op, rec)

    elif args['plot_predictions']:
        tub = args['--tub']
        model = args['--model']
        plot_predictions(cfg, tub, model)

    elif args['custom_train']:
        tub = args['--tub']
        model = args['--model']
        custom_train(cfg, tub, model)





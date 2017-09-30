#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car and train a model for it. 

Usage:
    manage.py (drive) [--model=<model>]
    manage.py (train) [--tub=<tub1,tub2,..tubn>] (--model=<model>)
    manage.py (calibrate)
    manage.py (check) [--tub=<tub1,tub2,..tubn>] [--fix]
"""


import os
from docopt import docopt
import donkeycar as dk 


def drive(model_path=None):
    #Initialized car
    V = dk.vehicle.Vehicle()
    cam = dk.parts.PiCamera(resolution=cfg.CAMERA_RESOLUTION)
    V.add(cam, outputs=['cam/image_array'], threaded=True)
    
    #Joystick pilot below is an alternative controller.
    #Comment out the above ctr= and enable the below ctr= to switch.
    #modify max_throttle closer to 1.0 to have more power
    #modify steering_scale lower than 1.0 to have less responsive steering
    ctr = dk.parts.JoystickPilot(max_throttle=cfg.JOYSTICK_MAX_THROTTLE,
                                 steering_scale=cfg.JOYSTICK_STEERING_SCALE)

    V.add(ctr, 
          inputs=['cam/image_array'],
          outputs=['user/angle', 'user/throttle', 'user/mode', 'recording', 'brake'],
          threaded=True)
    
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
            print("pilot_angle:", pilot_angle)
            return pilot_angle, user_throttle
        
        else:
            print("pilot_angle:", pilot_angle)
            print("pilot_throttle:", pilot_throttle)
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
      
      if target_throttle != 0.0:
        print("measured_throttle:", measured_throttle)
        print("target_throttle:", target_throttle)

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

      if pid_throttle != 0.0:
        print("pid_throttle:", pid_throttle)

      return pid_throttle

    throttle_with_pid_part = dk.parts.Lambda(throttle_with_pid)
    V.add(throttle_with_pid_part,
          inputs=['target_throttle','pid/output'],
          outputs=['pid_throttle'])

    steering_controller = dk.parts.PCA9685(cfg.STEERING_CHANNEL)
    steering = dk.parts.PWMSteering(controller=steering_controller,
                                    left_pulse=cfg.STEERING_LEFT_PWM, right_pulse=cfg.STEERING_RIGHT_PWM)
    
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



def train(cfg, tub_names, model_name):
    
    X_keys = ['cam/image_array']
    y_keys = ['user/angle', 'measured_throttle']
    
    def rt(record):
        record['user/angle'] = dk.utils.linear_bin(record['user/angle'])
        return record

    kl = dk.parts.KerasCategorical()
    
    if tub_names:
        tub_paths = [os.path.join(cfg.DATA_PATH, n) for n in tub_names.split(',')]
    else:
        tub_paths = [os.path.join(cfg.DATA_PATH, n) for n in os.listdir(cfg.DATA_PATH)]
    tubs = [dk.parts.Tub(p) for p in tub_paths]

    import itertools

    gens = [tub.train_val_gen(X_keys, y_keys, record_transform=rt, batch_size=cfg.BATCH_SIZE, train_split=cfg.TRAIN_TEST_SPLIT) for tub in tubs]


    # Training data generator is the one that keeps cycling through training data generator of all tubs chained together
    # The same for validation generator
    train_gens = itertools.cycle(itertools.chain(*[gen[0] for gen in gens]))
    val_gens = itertools.cycle(itertools.chain(*[gen[1] for gen in gens]))

    model_path = os.path.join(cfg.MODELS_PATH, model_name)

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
    if tub_names:
        tub_paths = [os.path.join(cfg.DATA_PATH, n) for n in tub_names.split(',')]
    else:
        tub_paths = [os.path.join(cfg.DATA_PATH, n) for n in os.listdir(cfg.DATA_PATH)]

    tubs = [dk.parts.Tub(p) for p in tub_paths]

    for t in tubs:
        tubs.check(fix=fix)

if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()
    
    if args['drive']:
        drive(cfg, model_path = args['--model'])
    
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





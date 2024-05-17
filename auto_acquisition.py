import subprocess
import datetime

#Define study name and date
study_name = "light_source_preset_Off_on"

machine_config = "cpl-cpl"

#get the current date with the format YYMMDD with only two digits for the year
current_date = datetime.datetime.now().strftime("%d%m%y")

# Machine name
machine_name = "PLI10"

# Define lists of exposure, gain, and gamma values
exposures = [11000]
gains =  [1,2,3,4,5,6,7,8,9]
gammas = [1,1.5,2,2.5,3,3.5,3.999]

# Use in case of pli10-get.py

# light_source_preset= "Off"
# light_source_preset= "Daylight5000K"
# light_source_preset= "Daylight6500K"
light_source_preset= ["Off"]

# direction
# direction = "cw"
direction = "ccw"

# Select camera model

# model_name = "acA5472-17uc" # 8 bit
# model_name = "a2A4504-18ucBAS" # 12 bit
model_name = ["acA5472-17uc","a2A4504-18ucBAS"]

# Acquisition index
# acq_index = 1

# Base path
# base_path = f"/Users/vsahu/Downloads/DATA/machine_studies/PLI10/studies{current_date}/light_source_preset/{machine_config}/{direction}/{model_name}/{light_source_preset}/"

# Calibrate the camera

# calibrate = True
calibrate = False


if calibrate==True:
    command = f"python pli10-get.py --calibrate --model_name {model_name} "
    subprocess.run(command, shell=True)

else:
    for light_source_preset in light_source_preset:
        for model_name in model_name:
            # 
            base_path = f"/Volumes/naat_sahu/machine_studies/PLI10/studies{current_date}/light_source_preset/{machine_config}/{direction}/{model_name}/{light_source_preset}/"   
            # Create a directory to store the acquired images
            subprocess.run(f"mkdir -p {base_path}", shell=True)

            acq_index = 1

            # create a markdown to store the executed commands and create the directory if it does not exist
            f = open(f"{base_path}{study_name}_{current_date}.md", "w")
            f.write(f"# Commands executed on {current_date}\n")
            f.write(f"Study Name: {study_name}\n")
            f.write(f"Machine Name: {machine_name}\n")
            f.write(f"Base Path: {base_path}\n")




            # Iterate over combinations of exposure, gain, and gamma
            for exposure in exposures:
                for gain in gains:
                    for gamma in gammas:
                        
                        # Generate the command with the current values
                        command = f"python pli10-get.py --acquire --base_path {base_path}acq{acq_index}_{exposure}_{gain}_{gamma} --n_angles 36 --n_polarisers 1 --n_stepper_steps 800 --n_large_gear_teeth 96 --n_small_gear_teeth 42 --model_name {model_name} --color_mode Mono8 --gain {gain} --exposure {exposure} --gamma {gamma} --light_source_preset {light_source_preset}"
                        # command = "python pli-get.py --calibrate"
                        
                        # Execute the command
                        try:
                            subprocess.run(command, shell=True, check=True)
                            # Write the command to the markdown file
                            f.write(f"acq{acq_index}\n{command}\n\n")
                            print("Command executed successfully:", command)
                            acq_index+=1
                        except subprocess.CalledProcessError as e:
                            print("Error executing command:", e)
                            # Handle error here if needed
                        

            f.close()

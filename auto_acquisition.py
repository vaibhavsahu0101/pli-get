import subprocess
import datetime

#Define study name and date
study_name = "light_source_preset_Off_on"

#get the current date
current_date = datetime.datetime.now().strftime("%Y%m%d")

# Machine name
machine_name = "PLI10"

# Define lists of exposure, gain, and gamma values
exposures = [ 11000]
gains =  [3]
gammas = [1]

# Use in case of pli10-get.py

light_source_preset= "Off"
# light_source_preset= "Daylight5000K"
# light_source_preset= "Daylight6500K"


acq_number = 1

# Base path
base_path = "/Users/vsahu/Downloads/DATA/machine_studies/PLI10/studies070524/light_source_preset/8_bit/OFF/"

# Create a directory to store the acquired images
subprocess.run(f"mkdir -p {base_path}", shell=True)

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
            command = f"python pli10-get.py --acquire --base_path {base_path}acq{acq_number}_{exposure}_{gain}_{gamma} --n_angles 36 --n_polarisers 1 --n_stepper_steps 800 --n_large_gear_teeth 96 --n_small_gear_teeth 42 --color_mode Mono8 --gain {gain} --exposure {exposure} --gamma {gamma} --light_source_preset {light_source_preset}"
            # command = "python pli-get.py --calibrate"
            
            # Execute the command
            try:
                subprocess.run(command, shell=True, check=True)
                # Write the command to the markdown file
                f.write(f"acq{acq_number}\n{command}\n\n")
                print("Command executed successfully:", command)
                acq_number+=1
            except subprocess.CalledProcessError as e:
                print("Error executing command:", e)
                # Handle error here if needed
            

f.close()

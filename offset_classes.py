import os

# Authored by Henry Lam and Jason Hamechand
# NOTE - ONLY WORKS FOR YOLOV5 FORMATS

def modify_first_integer(line, amount):
    # Find the position of the first integer in the line
    first_digit_pos = None
    for i, char in enumerate(line):
        if char.isdigit():
            first_digit_pos = i
            break
    if first_digit_pos is None:
        return line  # No integer found, return the original line

    # Extract the first integer and add the amount to it
    first_integer = ""
    for j in range(first_digit_pos, len(line)):
        if line[j].isdigit():
            first_integer += line[j]
        else:
            break
    new_first_integer = int(first_integer) + amount

    # Replace the first integer in the line with the new value
    new_line = line[:first_digit_pos] + str(new_first_integer) + line[first_digit_pos+len(first_integer):]
    return new_line

dir_path = os.path.dirname(os.path.realpath(__file__))

while(True):
    dataset = input("Enter the dataset's folder name in the current directory: ")
    dataset_path = os.path.join(dir_path, dataset)

    if not os.path.isdir(dataset_path):
        print(dataset_path, "is not a valid directory")
        continue

    offset = input("Enter the offset for the classes: ")
    check = ["train", "test", "valid"]
    for folder in check:

        #Get a list of all fie names in labels directory
        labels_path = os.path.join(dataset_path, r"{}\labels".format(folder))
        labelfiles = os.listdir(labels_path)

        # Loop through all label files
        for labelfile in labelfiles:
            input_file = os.path.join(labels_path, labelfile)

            with open(input_file, 'r+') as f:
                lines = f.readlines()  # Read all lines in the file into a list
                f.seek(0)  # Reset the file pointer to the beginning of the file

                for line in lines:
                    modified_line = modify_first_integer(line.rstrip('\n'), int(offset))
                    f.write(modified_line + '\n')  # Write the modified line back to the file

                f.truncate()  # Truncate the file in case the new content is shorter than the original
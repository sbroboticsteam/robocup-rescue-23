import os

# Authored by Henry Lam and Jason Hamechand
dir_path = os.path.dirname(os.path.realpath(__file__))

while(True):
    dataset = input("Enter the dataset's folder name in the current directory: ")
    dataset_path = os.path.join(dir_path, dataset)

    if not os.path.isdir(dataset_path):
        print(dataset_path, "is not a valid directory")
        continue

    check = ["train", "test"]
    for folder in check:
        # Get a list of all file names in images directory
        images_path = os.path.join(dataset_path, r"{}\images".format(folder))

        imagefiles = os.listdir(images_path)
        images = [] 

        # Remove file suffix
        for file in imagefiles:
            images.append(file.removesuffix('.jpg')) 

        #Get a list of all fie names in labels directory
        labels_path = os.path.join(dataset_path, r"{}\labels".format(folder))

        labelfiles = os.listdir(labels_path)
        label = [] 

        # remove file suffix
        for file in labelfiles:
            label.append(file.removesuffix('.txt')) 

        #Common Items between the two arrays
        notInLabel = [item for item in images if item not in label]
        print("Prints the items that are not in labels folder:",notInLabel)

        notInImages = [item for item in label if item not in images]
        print("Prints the items that are not in images folder:",notInImages)

        notSharedItems = notInLabel + notInImages
        print(f"Amount of unshared items in {folder}:",len(notSharedItems))
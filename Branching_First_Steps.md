## Download Git 
[Download Git from here](https://git-scm.com/downloads)

## Cloning the RCR Repository

Open your preferred terminal (Cmd, Powershell, Bash) and run the following command in your desktop directory.
``` bash
git clone https://github.com/sbroboticsteam/robocup-rescue-23.git
```

Enter your robocup-rescue-23 directory with the following command
``` bash
cd robocup-rescue-23
```
## Create Your Branch

Right now you only have the master branch, use this following command to pull the development branch:
``` bash
git checkout -b development origin/development
```

You are now working in the most recent version of the development branch. Now you are going to create your own branch, which will be the only branch you are working on. Run the following command:
``` bash
git checkout -b NAME_OF_BRANCH
```
Replace `NAME_OF_BRANCH` with a more appropriate name. A name the concisely summarizes the feature/functionality being added will suffice.

## Fetching and Merging Updates
In order to ensure our code is up-to-date with the development branch, we will need to run the following commands. First, we will fetch code from the public RCR repository to see whether updates have been made.

``` bash
git fetch origin
```
If updates have been made to one or more branches, a list of those branches will be displayed in the terminal. If you see a branch of interest you want to obtain updates from (usually `origin/development` or `origin/main`), you then merge said branch into yours by running:
``` bash
git merge origin/BRANCH_OF_INTEREST
```
## Set Upstream Branch

"Set upstream branch" means telling your current branch which other branch to reference when updating itself with other people's code.

This is useful because you want to ensure the files you are working with are not outdated. To set a branch (the one you are developing the new feature/functionality on) upstream to the development branch, do the following:
``` bash
git branch --set-upstream-to origin/development
```

Now let's check that the setup everything is correct with the following command:
``` bash
git branch -vv
```

Your terminal should look like this:

<img src="misc\img\gitBranchingCheck.JPG" alt="Kitten" title="Git Branching Check" style="border-radius : 7px" />

This is the layout of the Git branches and an example of the workflow we will try to follow:
<img src="./misc/img/gitBranchWokflow.png" alt="Kitten" title="Git Branching Check" style="border-radius : 7px" />
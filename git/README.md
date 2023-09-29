<h1 align="center">Git Overview</h1>
<h3 align="center">CRC Software Team</h3>

<br>
  <p align="center">
    GitHub Repo
    <br/>
    <a href="https://github.com/CabrilloRoboticsClub/tiny_hawk"><strong>Tiny Hawk »</strong></a>
    <br/><br/>
  </p>
<br>

<details>
<summary>Table of contents</summary>
    <ul>
        <li><a href="#what-are-git-and-github">What are Git and GitHub</a>
            <ul>
                <li><a href="#recording"> Recording</a>
                <li><a href="#slides"> Slides</a>
            </ul>
        <li><a href="#using-git-and-git-hub">Using Git and GitHub</a>
            <ul>
                <li><a href="#0%EF%B8%8F⃣-installing-git-cygwin"> Installing git (Cygwin)</a>
                <li><a href="#0%EF%B8%8F⃣-installing-git-mac-and-linux"> Installing git (Mac and Linux)</a>
                <li><a href="#1%EF%B8%8F⃣-create-a-github-account"> Create a GitHub account</a>
                <li><a href="#2%EF%B8%8F⃣-set-up-ssh-cygwin"> Set up SSH (Cygwin)</a>
                <li><a href="#2%EF%B8%8F⃣-set-up-ssh-mac-and-linux"> Set up SSH (Mac and Linux)</a>
                <li><a href="#3%EF%B8%8F⃣-generate-an-ssh-key"> Generate an SSH key</a>
                <li><a href="#4%EF%B8%8F⃣-add-the-ssh-key-to-github"> Add the SSH key to GitHub</a>
                <li><a href="#5%EF%B8%8F⃣-fork-the-repo-to-your-account"> Fork the repo to your account</a>
                <li><a href="#6%EF%B8%8F⃣-clone-your-forked-repo-to-your-system"> Clone your forked repo to your system</a>
            </ul>
        <li><a href="#maintaining-your-repo">Maintaining your repo</a>
            <ul>
                <li><a href="#0%EF%B8%8F⃣-sync-and-pull-the-changes">Sync and pull the changes</a>
                <li><a href="#1%EF%B8%8F⃣-make-a-branch">Make a branch</a>
                <li><a href="#2%EF%B8%8F⃣-commit-your-changes">Commit your changes</a>
                <li><a href="#3%EF%B8%8F⃣-push-your-changes">Push your changes</a>
                <li><a href="#4%EF%B8%8F⃣-create-a-pull-request">Create a pull request</a>
            </ul>
    </ul>
</details>

<br>

# What are Git and GitHub
## Recording 
*Click on the image to open the [video](https://youtu.be/hzFErKeImTs)*
[![Watch the video](https://i.imgur.com/ytJSMpi.png)](https://youtu.be/Opt-DSF9d_Y?si=3k5_N0q704sF5LPl "Tiny Hawk: What are Git and GitHub")

<br>

## Slides
*Click on the image to open the [slides](https://www.canva.com/design/DAFurH8Zj8M/pdIT0beDIdiFNPjTMQ6ZKQ/view?utm_content=DAFurH8Zj8M&utm_campaign=designshare&utm_medium=link&utm_source=publishsharelink)*
[![Slides](https://i.imgur.com/fXF5XmL.png)](https://www.canva.com/design/DAFurH8Zj8M/pdIT0beDIdiFNPjTMQ6ZKQ/view?utm_content=DAFurH8Zj8M&utm_campaign=designshare&utm_medium=link&utm_source=publishsharelink "Tiny Hawk: What are Git and GitHub")

<br>

# Using Git and GitHub
## 0️⃣ Installing git (Cygwin)
If you are on Windows, you have probably already installed Cygwin on your device. When you were installing, Cygwin gave you the option to install additional packages, one of these was for Git. Chances are you did not install this, but that is okay we can add packages after installation (kinda). There is no package management in Cygwin outside of the setup program, so you will have to run setup-x86_64.exe again. 
1. Open `File Explore` then to `Downloads` on your device
2. Search for `setup-x86_64.exe` and open the file
3. Go through the set up process again (choose the same options as last time) until you reach the `Select Packages` page. Here use the search tool to search for "Git". Choose the package named `Git` with description `Distributed version control system`
    ![packages](https://github.com/steph1111/F23_CS11_SI/assets/96219204/05954a94-ad7e-4596-855d-8a042905df34)
    Note: now is a good time to install any additional packages you may want like Vim
4. Finish the rest of the set up process
5. Open your Cygwin terminal and confirm git has been properly installed by running
    ```sh
    git --version
    ```
    You should get back something that looks like `git version 2.39.0`

<br>

## 0️⃣ Installing git (Mac and Linux)
1. By default, git should be installed on Mac and Linux systems
2. Confirm git is installed by running the following command in your terminal
    ```sh
    git --version
    ```
    You should get back something that looks like `git version 2.39.0`
3. If you did not get a version number, follow the this guide: [Install Git](https://github.com/git-guides/install-git)

<br>

## 1️⃣ Create a GitHub account
GitHub is the free web based platform we and many other software engineers use to share and collaborate on projects as well as control version history.
<img width="1440" alt="Screen Shot 2023-09-10 at 7 39 35 AM" src="https://github.com/steph1111/F23_CS11_SI/assets/96219204/0ed2d5d9-7b63-4953-8a56-3bfe7901db79">


1. Go to [GitHub](https://github.com/) and follow the steps to create an account
	- As a Cabrillo student, you can get a bunch of free goodies from GitHub Education. For now, you should continue following this guide, but later, don't forget to go to [education.github.com](https://education.github.com/). You will have to add your `STUDENTID@student.cabrillo.edu` email (it's a Gmail account).
2. Return to the terminal (cygwin or your standard terminal) to configure git
3. Set up your username using the following command where `<your_GitHub_username>` is the username you chose when creating your GitHub account
    ```sh
    git config --global user.name <your_GitHub_username>
    ```
4. Set up your email using the following command where `<your_GitHub_email>` is the email you used when creating your GitHub account
    ```sh
    git config --global user.email <your_GitHub_email>
    ```

<br>

## 2️⃣ Set up SSH (Cygwin)
1. Open Cygwin as Administrator and enter the following command to configure SSH
    ```sh
    ssh-host-config
    ```
2. Enter yes for the prompts 
3. You will be prompted to with the text below
    <div><pre style="margin: 0; line-height: 125%"><span style="color: #DA70D6">***Query:</span><span style="color: white"> Enter the value CYGWIN for the daemon: []</span></pre></div>

    For the value enter `ntsec`
    <div><pre style="margin: 0; line-height: 125%"><span style="color: #DA70D6">***Query:</span><span style="color: white"> Enter the value CYGWIN for the daemon: [] ntsec</span></pre></div>

<br>

## 2️⃣ Set up SSH (Mac and Linux)
1. SSH should already be on Mac and Linux systems

<br>

## 3️⃣ Generate an SSH key
1. Generate an SSH key to connect your system to your GitHub account
    ```sh
    ssh-keygen
    ```
    - When it asks for the file location, press enter to save it to the default location (`~/.ssh`)
    - Hit enter when it asks for the password to not save a password or enter one if you think you can remember it
2. Copy the generated ssh key
    ```sh
    cat ~/.ssh/id_rsa.pub
    ```

<br>

## 4️⃣ Add the SSH key to GitHub
1. Return to GitHub and open your [settings](https://github.com/settings/) by clicking on your icon then ⚙️ `settings`
2. Scroll to  🔑 `SSH and GPG keys`

   <img width="300" alt="Tabs" src="https://github.com/steph1111/F23_CS11_SI/assets/96219204/43005ec1-80b4-4a11-8659-41eff2ee83b7">

4. Click the <img width="80" alt="New SSH Key" src="https://github.com/steph1111/F23_CS11_SI/assets/96219204/7529cd77-805d-4df4-a24d-e10786648852"> button located in the top right corner.
1. Choose a name for the key (something such that you'll remember what computer/system it was for)
2. Paste the key from step 2 into the `Key` box
3. Save the key by clicking <img width="80" alt="Add SSH Key" src="https://github.com/steph1111/F23_CS11_SI/assets/96219204/cb49a8b5-7d60-4498-9595-57634f3f6bba">

<br>

## 5️⃣ Clone the repository to your system
1. Open the `tiny_hawk` [repository](https://github.com/CabrilloRoboticsClub/tiny_hawk/). At the top of the page click the green <img width="70" alt="code" src="https://github.com/steph1111/F23_CS11_SI/assets/96219204/e6ab5cd8-2adf-44bd-8dab-865fefea6429">
 button. Open the SSH tab and copy the link

  <img width="364" alt="Screen Shot 2023-09-10 at 6 50 51 PM" src="https://github.com/steph1111/F23_CS11_SI/assets/96219204/477b25d6-1944-47e7-a0ea-507461b5d0cb">

2. Return to your terminal and navigate using `cd` to where you would like this repo's contents to live with your file system (Perhaps inside a Documents folder?)
3. To clone the repo use the `git clone` command and paste the link from step 1. This points to the existing repo on GitHub and makes a connected copy, or clone, on your system. 
    ```sh
    git clone git@github.com:CabrilloRoboticsClub/tiny_hawk.git
    ```
4. To confirm the clone was successful list your files with `ls`. You should see the name of the cloned repo in your current directory.

<br>

# Maintaining your repo

## 0️⃣ Sync and pull the changes
1. Open your forked repo in GitHub. If there are no changes you should see a message that looks like the following stating this branch is up to date. If this is the case it means I have not made changes and you can ignore the following steps.
    ![up_to_date](https://github.com/steph1111/F23_CS11_SI/assets/96219204/a67537ba-91e8-4ec6-907d-8181b769b281)

2. If there are upstream changes to merge there should be a message on this page similar to the following:
    ![x_commits_behind](https://github.com/steph1111/F23_CS11_SI/assets/96219204/786f9c39-25de-4fd2-8b2b-2012b332646d)
   This means there are changes to the upstream repo that need to be synced to your fork
3. Click on the button titled 🔄 `Sync fork`, you should see a window stating the code is out of date. Click <img width="80" alt="update branch" src="https://github.com/steph1111/F23_CS11_SI/assets/96219204/3330cd21-db4f-4ed4-9803-36796deea681">

4. Next navigate to your fork on your system 
5. Enter the command `git pull` to pull the changes on GitHub onto your device
   ```sh
   git pull
   ```

## 1️⃣ Make a branch
1. Pull the meta data from the remote repo
    ```sh
    git fetch
    ```
2. Use the following command to see if there is an active branch for the changes you are planning on making
    ```sh
    git branch -r
    ```
    If so checkout that branch and work on your code with 
    ```sh
    git checkout remote_branch
    ```
    Where `remote_branch` is the name of the remote branch to checkout, i.e. `origin/remote_branch` would just be `remote_branch`
3. If there is no branch dedicated for what you want to change, create one where `branch_name` is a descriptive name of the branch
    ```sh
    git checkout -b branch_name
    ```
4. Write your changes

<br>

## 2️⃣ Commit your changes
Once you make some changes you would like to be documented you need to `commit` them. Committing changes is how you can mark versions of your code you would like to be tracked. It is a good idea to commit changes before and after you add/remove a feature that way there is a record of your changes. Also commit changes after you have finished a session of programming then `push` (see following section)
1. To commit changes you use the `git commit` command. The parts of the command are as follows:
     - `git`: Denotes we are using a git command 
     - `commit`: The name of the command is commit, this is what we would like to do
     - `-m`: This is a flag that says we would like to add a commit message
     - `"Message here`: Between quotes state what changes you are committing 
     - `file_name`: Replace this with the file you wish to commit
     
     All together the git commit command syntax looks like this:
     ```sh
     git commit -m "Message here" file_name
     ```
     An example commit may be
     ```sh
     git commit -m "Added endl to move to the next line" hello.cpp
     ```
2. You will receive a short message summarizing the changes

<br>

## 3️⃣ Push your changes
Once you have finished a coding session it is a good idea to `push` your commits. The `push` command is basically the opposite of `pull`. `push` takes the changes (commits) you made on your local system and sends them to GitHub. <span style="color:red"> **IMPORTANT: always push before you pull!**</span> 
1. Commit your changes (see above)
2. Enter the command `git push origin branch_name` to push the changes from your system to GitHub where `branch_name` is the name of the branch you are on
3. You can confirm this worked by heading to Github and seeing the changes reflected on your repo page
   
<br>

## 4️⃣ Create a pull request
When you think your code is ready to merge into the main Cabrillo Robotics Code create a pull request. 
1. Go to the pull requests page on GitHub and click on <img width="80" alt="new pull request" src="https://i.imgur.com/EXaY4og.png">
2. Select your branch as the branch to compare to
3. Add a description of what changes you made and why

---
title: Environment setup
date: 2016-10-15 14:29:38
tags:
---
The first thing I can do without doing any research is to set up my environment.
I know I will probably change my mind as things progress.

### Things I did
I installed stock Ubuntu in a virtual machine, as I already had the disc image downloaded.
It was very easy, but I really didn't, and still don't like the Unity desktop environment.
It also comes preinstalled with a lot of things I never use.

I opted for Xubuntu, which uses XFCE, and I made a physical partition. I'm pleased with the results.
I will continue to use Sublime Text 3 as my text editor, and a customised `zsh` shell environment.

I will be keeping my project files under source control.
I used mercurial a lot on placement and it seems easier than git, so once things get rolling I'll be doing this.
I know how to use virtual environments and pip with python, but I won't do this until I know I need it.
I'll be trying to take regular screenshots/photos to stick in here.

### Markdown
I need any notes to be portable between computers and operating systems (This means no Microsoft Office!)
Microsoft Office provide an online version of Office which is surprisingly good. But I'll have to worry about logging in, inserting media into the documents and no organisation between documents beside what I can do with folders.
I already have an excellent Markdown editor (Sublime Text) which doubles for code.
I know the GitHub 'about' pages are written in Markdown, so it must have good support for inserting code snippets.

After looking around, I'm installing `markmon`, an automatic markdown preview tool, which generates platform-independent HTML on-the-fly as I write.

- Install Package Control for Sublime Text
- Install `markmon` to autogenerate the markdown preview
	Has some external dependencies:
	```zsh
	$ sudo ln -s /usr/bin/nodejs /usr/bin/node
	$ sudo apt-get install npm
	$ sudo npm install -g markmon
	$ sudo apt-get install pandoc
	```
- I've added a keyboard shortcut `alt+m` to serve and bring up the HTML page.
	Markmon works on Windows, but doesn't appear to be able to use Cygwin.
	I had to go into the markmon settings and modify them like this:
	```json
	"executable": "markmon.cmd"
	```

Here's a screenshot of it in action:

![These notes are now out of date!](/Robotic-Telepresence/2016/10/15/Environment-setup/Markmon.png)

Using [pandoc](http://pandoc.org/), one can easily convert between many formats, including HTML, docx, LaTeX etc. I may use this to insert parts of my notes into my reports.

There are a huge number of purely online editors, such as [Stackedit](https://stackedit.io/editor)
Some of them look very good, but I would not have full control over the files.

### Other things

In the last project meeting, Rich asked about how the graphics are drawn to the headset. I didn't give a clear answer, but here it is:
- Direct mode: Graphics are drawn directly to the HMD (head-mounted display). Better VR (virtual reality) performance.
- Extended mode: HMD display is treated like an additional monitor.
- OpenVR: **Proprietary** VR abstraction layer which supports a number of HMDs.
- OSVR: **Open source** abstraction layer which supports a number of HMDs (including the Vive).
- OpenHMD: Supports some HMDs but not the Vive.

(Info found at [vronlinux](http://www.vronlinux.com/articles/adventures-with-openvr-and-the-vive-on-linux.8))
([This one](https://www.gamingonlinux.com/articles/first-steps-with-openvr-and-the-vive-on-linux.7229) is a slightly more up to date version)

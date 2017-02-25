---
title: Jekyll site
date: 2016-10-26 18:35:15
tags:
---
After some research I've found a possibly good solution.
Currently I'm keeping my notes as markdown on a USB stick under source control (Mercurial).
If I push them to my bitbucket repository, they can be hosted under the URL:
> **USERNAME**.bitbucket.io

The page can host anything static. No PHP, but you can do a lot with JS and CSS.
Set up the `Aerobatic` BitBucket plugin. It's supposed to support jekyll, so that I can just push new Markdown to the repo and it will rebuild the site for me.

I've got jekyll to work, however I've found out that the Aerobatic plugin for BitBucket only has 5 publishes per 24 hours.
I'm going to see if I can run it locally to preview before pushing, but this could be a dealbreaker.
I've got jekyll to work locally.
I can't get the automatic updating when a file is edited to work, but it doesn't matter so much.

Currently my notes are one big markdown file, I'm going to split them into individual posts.

I've run into some problems with the index page.
Aerobatic couldn't find the index.html.
It's not in the jekyll folder in the repo, I assumed everything would be OK seeing as though it served locally well.
It turns out the problem was that local Jekyll was using `index.md`, and Aerobatic was _specifically_ looking for `index.html`

Aerobatic won't populate the index.html either, it needs to work on `.md` files. Hmm. Will sort it out another time.

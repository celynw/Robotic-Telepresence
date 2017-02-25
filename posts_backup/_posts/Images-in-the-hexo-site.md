---
title: Images in the hexo site
date: 2016-11-05 21:55:44
tags:
---
I've been having problems showing images in the generated html.
I've added this line in the main configuration:
```yml config.yml
post_asset_folder: true
```
This means that for each post created, an extra empty directory with the same name as the post is created alongside.
It also makes sense to keep the images with their respective post and not mashed together in a single folder.

I'm also having to put the absolute path for the images in the markdown, because they don't display on the index page.

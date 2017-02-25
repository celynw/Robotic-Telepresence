---
title: Named shared memory
date: 2016-12-02 10:56:53
tags:
---
There seem to be more easily found resources for these.
- Memory is broken into pages, so buffer space is rounded to the next available size

In order for the data to be accessible globally, you must set the `Global` flag in its name:
```cpp
TCHAR szName[] = TEXT("Global\\MyFileMappingObject");
```
This requires Administrator privileges (`SeCreateGlobalPrivilege, SE_CREATE_GLOBAL_NAME`).
Running Visual Studio as administrator is supposed to make it so that any program run in the debugger is also run with privileges.
This was true, as the return value from `GetLastError()` went from `5` (cannot access) to `2` (no idea).
I couldn't solve this, but running the built executables outside as administrator worked as expected.

While I'm on the subject of learning how Visual Studio actually works, I've repackaged and renamed each project and put them all into one solution, which accesses a single instance of the built `ros_lib` library for `rosserial_windows`.

I was just trying to create a file mapping in a Unity script...
> error CS0234: The type or namespace name `MemoryMappedFiles` does not exist in the namespace `System.IO`

System.IO.MemoryMappedFiles is only available in .NET 4.0 or later, and Unity is still using .NET 3.5 (.NET 2.0 CLR), not likely to be updated for a long time. Bummer.

It looks like there could be a wrapper, [filemap](https://github.com/tomasr/filemap/tree/master).
I got it to compile, and I'm left with a `.dll`. I'm not sure how to use it (the repository has no documentation) and I'm pretty sure I'll have to edit it to make it compaitble with Unity so I'm not very hopeful.

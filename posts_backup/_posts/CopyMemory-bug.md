---
title: CopyMemory bug
date: 2017-03-15 00:30:28
tags:
---
I had a bug for many many hours, took me a while to narrow down.
I verified that everything worked perfectly at home.
However, in the lab, there was some strange behaviour with the shared memory for the hand poses.
`CopyMemory` is defined on [MSDN](https://msdn.microsoft.com/en-us/library/windows/desktop/aa366535) like this:
```cpp CopyMemory definition
void CopyMemory(
  _In_       PVOID  Destination,
  _In_ const VOID   *Source,
  _In_       SIZE_T Length
);
```

I encapsulated the memory functions in a class for my ROS node.
```cpp Single byte reading function
uint8_t FileMapping::read_data(int offset) {
	CopyMemory(data, buffer+offset, 1);
	return data[0];
}
```

However, to get things to work correctly in the lab, I had to half the offset parameter.
```cpp Modified line
	CopyMemory(data, buffer+(offset/2), 1);
```

This problem also occurs in a `float FileMapping::read_data_float(int offset)` function.
I've been careful to use `sizeof(float)` etc. and not assume sizes of things (although I have checked, and it is defined as 4 bytes everywhere).

This fix works so I'll leave it as this, and this is the first place to look when the problem surfaces again.

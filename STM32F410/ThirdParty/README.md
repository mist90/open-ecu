# Heapless console dependencies

Unmodified upstream sources, used only by `Core/Src/console_libc.cpp`:

* [nanoprintf](https://github.com/charlesnicholson/nanoprintf), v0.8.1,
  `bebaecbeaf430e36a2d30d31a2fed4e098388c93`; 0BSD / Unlicense (license at end of header).
* [fast_float](https://github.com/fastfloat/fast_float), v8.3.2,
  `42f254836ee416ffcf2f4e6576260003ac33d48c`; MIT / Apache-2.0 / Boost (license files included).

Retrieved from the GitHub archives at those exact revisions. SHA-256 of archives:

```
f8c4b7776f207ad7afce70c339d050462839c72335930ecc26e9e6a03aa6b4b4  nanoprintf.tar.gz
1e01958d0ca4671d95dea40722003f36003ecb10e164ba595795d71a2128f390  fast_float.tar.gz
```

The common console uses decimal/scientific numeric arguments, `inf`/`nan` (then
validated by its command handlers), `%f`, integer and string output. The F410
adapter supports those without newlib's hidden heap allocations; hexadecimal
floating-point input is not part of the AT protocol. Control arithmetic remains
single precision; variadic printf still obeys the C ABI's float promotion rule.

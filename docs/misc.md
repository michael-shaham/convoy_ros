# Miscellaneous

These notes will likely only be used by Michael for his benefit.

## Map caps lock to esc/ctrl

Easier method is to use the Gnome Tweaks tool.

```
sudo apt install gnome-tweaks
```

Then hit the super (Windows) key and search for `Tweaks` and open it. Go to the 
`Keyboard & Mouse` section, hit `Additional Layout Options`, then 
`Caps Lock behavior` and select `Make Caps Lock an additional Esc`.

Another option is to use `keyd`, as described below. This is copied from the 
[keyd GitHub page](https://github.com/rvaiya/keyd).

```bash
git clone https://github.com/rvaiya/keyd
cd keyd
make && sudo make install
sudo systemctl enable keyd && sudo systemctl start keyd
sudo vim /etc/keyd/default.conf
```

Paste

```
[ids]

*

[main]

# Maps capslock to escape when (double?) pressed and control when held.
capslock = overload(control, esc)
```

into the `default.conf` file. Then run

```sudo systemctl restart keyd```

one more time.
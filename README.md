# Discrete Kalman Filter

The project presented here consists in the implementation of a discrete Kalman filter, as done in this [link].

---
## Requirements
This project has been tested in Ubuntu (version 14.04), and needs the following dependencies: [g++], [eigen3] and [allegro4]. In order to install these dependencies in an Ubuntu machine, run the following commands in a terminal window (in this case, the terminal is opened in the folder where the project is located, e.g., `project-directory`):

```zsh
➜  project-directory    sudo apt-get install g++
➜  project-directory    sudo apt-get install libeigen3-dev
➜  project-directory    sudo apt-get install liballegro4.2‐dev
```

---
## Compilation instruction
Once that the above dependencies has been installed, the program can be compiled by means of the `makefile`. More precisely, it will be sufficient to run the following command in the terminal:

```zsh
➜  project-directory    make
```

---
## Run the program
After the compilation, the `makefile` can be used to run the program, as follows:

```zsh
➜  project-directory    make run
```

**Notes**

1. The program needs sudo privileges to be executed.

2. The `makefile` can also be used to clean the program, in this case just run the command:
    ```zsh
    ➜  project-directory    make clean
    ```

---
## Further informations
For a detailed description of the project structure see the [docs].


[allegro4]: http://liballeg.org/index.html
[docs]: https://github.com/cubocicloide/kalman/blob/master/docs.pdf
[eigen3]: http://eigen.tuxfamily.org/index.php?title=Main_Page
[g++]: https://www.cs.fsu.edu/~myers/howto/g++compiling.txt
[link]: https://www.cs.utexas.edu/~teammco/misc/kalman_filter/
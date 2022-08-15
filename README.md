# A-Maze-ing Race
This was a hobby project I worked on in the summer of 2022. It's a game implemented on an [Arduino Metro M0 Express](https://www.adafruit.com/product/1484) using an [32x32 RGB LED Matrix](https://www.adafruit.com/product/1484). The controls for the game are made using a [PS2 Thumb Joystick](https://leeselectronic.com/en/product/4270-ps2-thumb-joystick.html).

![A-Maze-ing Race](https://user-images.githubusercontent.com/5415113/184574291-e83e12ed-1282-40c9-8033-3af1ed1c1410.jpeg)

## Starting the Game
When you boot up the game, you can select which mode to choose.

### Game Mode
[<img src="https://user-images.githubusercontent.com/5415113/184574366-923122e3-3a55-4809-83e6-69e17c12f21e.jpg" width="300" />](https://user-images.githubusercontent.com/5415113/184574366-923122e3-3a55-4809-83e6-69e17c12f21e.jpg)

Choose between two play modes:
- [*Game*](#game) - play the default "Game"
- [*Cstm*](#custom-settings) - play with custom settings, without tokens or a timer

## Game
The object of the *Game* is to collect as many tokens as possible and escape the maze before time runs out. The maze is randomly generated every round, and the tokens are randomly placed. You get points and extra time for every token you collect, points if you escape the maze, and a time-bonus for the time you have left when you escape.

[<img src="https://user-images.githubusercontent.com/5415113/184574991-102e7a3b-14a6-4e5a-91d8-f020d9fdf572.jpeg" width=300 />](https://user-images.githubusercontent.com/5415113/184574991-102e7a3b-14a6-4e5a-91d8-f020d9fdf572.jpeg)


## Custom Settings
In this mode you can specify how you'd like to play the game. In all custom games, there are no tokens and no time-limit.
[<img src="https://user-images.githubusercontent.com/5415113/184574596-66785c50-1895-4bea-b496-ec3354b62889.jpeg" width=300 />](https://user-images.githubusercontent.com/5415113/184574596-66785c50-1895-4bea-b496-ec3354b62889.jpeg)


### Size

- S[mall] - play in a 9x9 maze
- M[edium] - play in a 19x19 maze
- L[arge] - play in a 29x29 maze

### Mist?
This setting controls whether you have a "shroud" which cloaks the maze walls, as you move through the maze the mist clears wherever you've visited.
- Y[es] - turn the mist on, revealing the walls when you visit them
- N[o] - turn off the mist, showing the maze walls at all times

### Viz
Controls the visibility inside the maze, indicated by a lit region around the player. This region also lifts the [mist](#mist) as the player visits an area.
- L[ow] - only light up 1 pixel around the player
- M[edium] - light up to a width of 4 pixels around the player
- H[igh] - light up to a width of 10 pixels around the player

## Colors
The game uses different colors to signify different types of pixels:
- White is the player color
- Red is for maze walls
- Blue is the start pixel
- Green is the exit pixel
- Yellow are hint pixels
- Purple is for time remaining (only in [Game](#game) mode)
- Cyan is for tokens (only in [Game](#game) mode)

## Acknowledgements
Thank you to my dad for helping me assemble the finished maze into a package, and thank you to the "A-Squad" for play-testing the game!

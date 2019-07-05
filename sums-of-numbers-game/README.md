## Sums of numbers game
- Run the `prepare_sum_objects.py` where the order of the sums to be played by each child will be saved as pickle files
- Run the main game with `python run_all_games.py`. By giving the argument *woz* the game enters in Wizard of Oz mode
- In Wizard of Oz mode, press the Space button to start the game or end it
- The game subscribes to the topic "athena.games.sums"

## IrisTK dialogue
- New states can be added that extend the *Dialog* state
- In order to make the agent speak, call the *agent_audio* state with the parameters: *p:audio="'path/to/wav_file.wav'"* and *p:agent="'system'"*
- In order to perform an animation with Zeno, call the *zeno_animation* state with the parameter: *"'path/to/animation.xml'"*

## Zeno Maven project
- Open the project `zeno/zeno_mvn` with NetBeans for easy building and running
- Change the string `hostName` to the IrisTK broker address
- Change the string `ipAddress` to the Zeno robot IP address
- How to add new actions:
  - The program subscribes to the topic "athena.zeno.behavior". In order to perform an action send an event to this topic with a respective message
  - Add a conditional statement like the one in line 115 in `ZenoInteraction.java` to control the added action

## GUI
- Inside `gui` folder. Two files, `tkNotebook.py` and `wizardOfOzRemote.py`
- Currently two Tabs (created at their respective functions): SumsTab for the sums-of-numbers-game and EmorecTab for the emotion-expression game
- To add a new Tab follow the same structure as the two functions
- There is the possibility to add Labels (with the `Label` class) and Buttons (with the `self.add_button` method)
- To run, execute in the terminal: `python wizardOfOzRemote.py`

## How to run
1) Copy the dialogue from dialogue/Data_CollectionFlow.xml to the respective IrisTK project folder
2) Compile the flow with the IrisTK compile tool
3) Open an IrisTK broker (`iristk broker`)
4) Set the broker address in the `config.py` file at the `broker` entry
5) Open the GUI by running `python wizardOfOzRemote.py` inside the `gui` folder
6) Run `python run_all_games.py`
7) Run the IrisTK flow from Eclipse
8) Open the `zeno/zeno_mvn` Maven project in NetBeans, build it and run it

Supported OpenSmile configuration files

We will host our own opensmile configuration files to limit the format options
of the output files.

I added two configuration files that will be used for the emotion classification
models.

To convert an arbitrary Opensmile configuration to a supported format, you need 
to:

1. Make sure that it reads the audio input from a cWaveSource
2. Make sure that it can output both a CSV and an ARFF file, using the
   following configuration

```
[arffsink:cArffSink]
reader.dmLevel=func
frameTime=0
frameIndex=0
filename=\cm[arffoutput(0){/tmp/features.arff}:name of WEKA Arff output file, set to a valid filename to enable this output sink]
relation=\cm[corpus{SMILEfeaturesLive}:corpus name, arff relation]
class[0].name = emotion
class[0].type = \cm[classes{unknown}:all classes for arff file attribute]
target[0].all = \cm[classlabel{?}:instance class label]
append=0

  ;;;; default (template) configuration section for component 'cCsvSink' ;;;;
[csvsink:cCsvSink]
reader.dmLevel = func
filename=\cm[csvoutput(0){/tmp/features.csv}:output csv file for LLD, disabled by default ?, only written if filename given]
delimChar = ,
append = 0
frameTime=0
frameIndex=0
printHeader = 1
```

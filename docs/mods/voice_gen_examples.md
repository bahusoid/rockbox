# Generating Voice Files with voice.pl for any existing Rockbox build

It uses the `voicestrings.zip` file that is included with every Rockbox build, located in `.rockbox/langs/` folder.

## Prerequisites

Make sure Perl and FFmpeg are installed. On Linux, run:
```bash
sudo apt install perl ffmpeg
```

Download a recent snapshot of the Rockbox source code and extract it:
https://download.rockbox.org/daily/source/rockbox-source.tar.xz

Here is a one-liner that downloads and extracts the sources into the `rb-voice-gen` folder and enters it:
```bash
DIR="rb-voice-gen"; mkdir -p "$DIR" && curl -sL https://download.rockbox.org/daily/source/rockbox-source.tar.xz | tar -xJ -C "$DIR" --strip-components=1 && cd "$DIR"
```

In the root of the Rockbox source folder, compile a minimal set of tools:
```bash
cd tools && make rbspeexenc wavtrim && cd -
```

## TTS Engines

The `voice.pl` script supports several TTS engines:

| TTS Engine  | Notes                                                                                                          |
| :---------- |:---------------------------------------------------------------------------------------------------------------|
| `flite`     |                                                                                                                |
| `espeak`    |                                                                                                                |
| `espeak-ng` | This is the preferred version of `espeak`.                                                                     |
| `festival`  |                                                                                                                |
| `mimic`     |                                                                                                                |
| `swift`     |                                                                                                                |
| `sapi`      | This engine is for Windows only (checks for `winver`)                                                          |
| `gtts`      | `gtts-cli` tool is required. You can install it using pip: `pip install gTTS-token gtts`.                      |
| `piper`     | You also need to set the `PIPER_MODEL_DIR` environment variable to the directory containing your Piper models. |
| `rbspeak`   |                                                                                                                |

## Usage

The basic syntax for using `voice.pl` is:

```bash
tools/voice.pl -B=<path_to_voicestrings_zip> -l=<language> -s=<tts_engine> -S='<engine_options>' -e=<encoder> -E='<encoder_options>' --wavtrim <path_to_wavtrim>
```

| Parameter          | Description                                                                                                                                                                                                                                                                                                                          |
| :----------------- |:-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `-B`               | Path to the `voicestrings.zip` file from your device's `.rockbox/langs/` directory.                                                                                                                                                                                                                                                  |
| `-l`, `--language` | The language to generate the voice file for (e.g., `english`, `german`).                                                                                                                                                                                                                                                             |
| `-s`, `--speaker`  | The TTS engine to use (e.g., `festival`, `espeak`).                                                                                                                                                                                                                                                                                  |
| `-S`, `--soptions` | Options to pass to the TTS engine. Use `-S=''` if the engine needs no options.                                                                                                                                                                                                                                                       |
| `-e`, `--encoder`  | The encoder to use. The default is `rbspeexenc`.                                                                                                                                                                                                                                                                                     |
| `-E`, `--eoptions` | Options to pass to the encoder. The default is `-q 7 -c 10`.                                                                                                                                                                                                                                                                         |
| `--wavtrim`        | Path to the `wavtrim` executable. This is required for trimming silence from generated WAV files.                                                                                                                                                                                                                                    |

You can also set the `POOL` environment variable to a directory path for caching generated voice files. This can significantly speed up subsequent runs. It's safe to use the same directory for different TTS engines or languages.  

### Results
After running the command, a `.voice` file (such as `english.voice`) will be generated in the current directory. Copy it to the `.rockbox/langs/` directory on your device.

## Examples

Here are some examples for different TTS engines. These examples assume you are in the root of the Rockbox source directory and that your `voicestrings.zip` file is located at `/path/to/device/.rockbox/langs/voicestrings.zip`. You will need to replace this with your actual path.

### Festival

**Installation**:

```bash
sudo apt-get install festival
```

**Command**:

Before running the `voice.pl` script with Festival, you need to create a `festival-prolog.scm` file in your build directory to specify the voice. This is the most reliable method to avoid shell syntax errors.

```bash
echo "(voice_kal_diphone)" > festival-prolog.scm
```

Then, run the `voice.pl` script:

```bash
tools/voice.pl -B=/path/to/device/.rockbox/langs/voicestrings.zip -l=english -s=festival -S='' -e=./tools/rbspeexenc -E='-q 7 -c 10' --wavtrim ./tools/wavtrim
```

**Command with Pool Directory**:

This example demonstrates using a pool directory to cache the generated `.wav` files by setting the `POOL` environment variable.

```bash
mkdir -p ~/rb-voices-pool
POOL=~/rb-voices-pool tools/voice.pl -B=/path/to/device/.rockbox/langs/voicestrings.zip -l=english -s=festival -S='' -e=./tools/rbspeexenc -E='-q 7 -c 10' --wavtrim ./tools/wavtrim
```

### eSpeak

**Installation**:

```bash
sudo apt-get install espeak
```

**Command**:

```bash
tools/voice.pl -B=/path/to/device/.rockbox/langs/voicestrings.zip -l=english -s=espeak -S='' -e=./tools/rbspeexenc -E='-q 7 -c 10' --wavtrim ./tools/wavtrim
```

### eSpeak-NG

**Installation**:

```bash
sudo apt-get install espeak-ng
```

**Command**:

```bash
tools/voice.pl -B=/path/to/device/.rockbox/langs/voicestrings.zip -l=english -s=espeak-ng -S='' -e=./tools/rbspeexenc -E='-q 7 -c 10' --wavtrim ./tools/wavtrim
```

### Flite

**Installation**:

```bash
sudo apt-get install flite
```

**Command**:

```bash
tools/voice.pl -B=/path/to/device/.rockbox/langs/voicestrings.zip -l=english -s=flite -S='' -e=./tools/rbspeexenc -E='-q 7 -c 10' --wavtrim ./tools/wavtrim
```

### gTTS

**Installation**:

```bash
pip install gTTS-token gtts
```

**Command**:

```bash
tools/voice.pl -B=/path/to/device/.rockbox/langs/voicestrings.zip -l=english -s=gtts -S='' -e=./tools/rbspeexenc -E='-q 7 -c 10' --wavtrim ./tools/wavtrim
```

### Piper

**Installation**:

Follow the installation instructions on the [Piper GitHub page](https://github.com/rhasspy/piper).

**Command**:

```bash
export PIPER_MODEL_DIR=/path/to/your/piper/models
tools/voice.pl -B=/path/to/device/.rockbox/langs/voicestrings.zip -l=english -s=piper -S='--model en_US-lessac-medium.onnx' -e=./tools/rbspeexenc -E='-q 7 -c 10' --wavtrim ./tools/wavtrim
```
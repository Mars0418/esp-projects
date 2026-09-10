$ErrorActionPreference='Stop'
$voice=New-Object -ComObject SAPI.SpVoice
$voice.Voice=$voice.GetVoices('Language=804').Item(0)
$voice.Rate=0
$audioDir=Join-Path $PSScriptRoot '../audio'
$phrases=Get-Content (Join-Path $audioDir 'texts.json') -Raw -Encoding UTF8 | ConvertFrom-Json
for($i=0;$i -lt $phrases.Count;$i++) {
    $stream=New-Object -ComObject SAPI.SpFileStream
    $stream.Format.Type=18
    $stream.Open((Join-Path $audioDir "$i.wav"),3,$false)
    $voice.AudioOutputStream=$stream
    $null=$voice.Speak($phrases[$i])
    $stream.Close()
}


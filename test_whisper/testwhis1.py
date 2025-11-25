import whisper

model = whisper.load_model("turbo")
result = model.transcribe("sample-1.mp3")
print(result["text"])



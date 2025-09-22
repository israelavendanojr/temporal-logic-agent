import ollama

model = "llama3.2"
prompt = "Are there any robotics opportunities in spain?"

response = ollama.generate(model=model, prompt=prompt)

print(response.response)


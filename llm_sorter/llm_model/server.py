from pydantic import BaseModel
import uvicorn
from fastapi import FastAPI

from typing import Optional
from model import Vicuna13B


class Item(BaseModel):
    prompt: Optional[str] = None
    image: Optional[str] = None


if __name__ == "__main__":
    model = Vicuna13B()
    app = FastAPI(debug=True)

    @app.post("/generate")
    def generate_plan(item: Item):
        print(f"-------------LLM INPUT-------------\n{item.prompt}")
        text = model.generate(item.prompt)
        print(f"-------------LLM OUTPUT-------------\n{text}")
        return {"text": text}

    @app.get("/name")
    def get_model_name():
        return {"name": model.name}

    uvicorn.run(app, host="0.0.0.0", port=8080)

"use client";

import { useState, useRef, useEffect } from "react";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Input } from "@/components/ui/input";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Avatar, AvatarFallback, AvatarImage } from "@/components/ui/avatar";
import { MessageCircle, X, Send, Bot, User, Minimize2, Maximize2 } from "lucide-react";

interface Message {
    role: "user" | "assistant";
    content: string;
}

export function ChatWidget() {
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState<Message[]>([
        { role: "assistant", content: "Hi! I'm your Physical AI assistant. Ask me anything about ROS 2, Isaac Sim, or the textbook content." }
    ]);
    const [inputValue, setInputValue] = useState("");
    const [isLoading, setIsLoading] = useState(false);
    const scrollRef = useRef<HTMLDivElement>(null);

    // Auto-scroll to bottom
    useEffect(() => {
        if (scrollRef.current) {
            // Simple timeout to allow DOM to update
            setTimeout(() => {
                const scrollContainer = scrollRef.current?.querySelector('[data-radix-scroll-area-viewport]');
                if (scrollContainer) {
                    scrollContainer.scrollTop = scrollContainer.scrollHeight;
                }
            }, 100);
        }
    }, [messages, isOpen]);


    const handleSendMessage = async () => {
        if (!inputValue.trim()) return;

        const userMsg: Message = { role: "user", content: inputValue };
        setMessages((prev) => [...prev, userMsg]);
        setInputValue("");
        setIsLoading(true);

        // Mock API call - Phase 3 will connect to FastAPI
        try {
            // simulate delay
            await new Promise(resolve => setTimeout(resolve, 1500));

            const botMsg: Message = {
                role: "assistant",
                content: "I'm currently a frontend demo. In Phase 3, I'll connect to the RAG backend to answer '" + userMsg.content + "' using the textbook content."
            };
            setMessages((prev) => [...prev, botMsg]);
        } catch (error) {
            console.error("Failed to send message", error);
        } finally {
            setIsLoading(false);
        }
    };

    return (
        <div className="fixed bottom-6 right-6 z-50 font-sans">
            {!isOpen && (
                <Button
                    onClick={() => setIsOpen(true)}
                    className="rounded-full w-14 h-14 bg-cyan-500 hover:bg-cyan-400 text-black shadow-lg shadow-cyan-500/20 transition-all hover:scale-110"
                >
                    <MessageCircle className="w-8 h-8" />
                </Button>
            )}

            {isOpen && (
                <Card className="w-[350px] md:w-[400px] h-[500px] flex flex-col bg-zinc-900 border-zinc-700 shadow-2xl animate-in slide-in-from-bottom-10 fade-in duration-300">
                    <CardHeader className="p-4 border-b border-zinc-800 flex flex-row items-center justify-between space-y-0">
                        <div className="flex items-center gap-2">
                            <Avatar className="w-8 h-8 bg-cyan-900/50 border border-cyan-500/30">
                                <AvatarFallback className="bg-transparent text-cyan-400"><Bot className="w-5 h-5" /></AvatarFallback>
                            </Avatar>
                            <div className="flex flex-col">
                                <CardTitle className="text-sm font-bold text-white">AI Assistant</CardTitle>
                                <span className="text-[10px] text-zinc-400 flex items-center gap-1">
                                    <span className="w-1.5 h-1.5 rounded-full bg-green-500 animate-pulse"></span>
                                    Online
                                </span>
                            </div>
                        </div>
                        <div className="flex items-center gap-1">
                            <Button variant="ghost" size="icon" className="h-6 w-6 text-zinc-400 hover:text-white" onClick={() => setIsOpen(false)}>
                                <Minimize2 className="w-4 h-4" />
                            </Button>
                            <Button variant="ghost" size="icon" className="h-6 w-6 text-zinc-400 hover:text-white" onClick={() => setIsOpen(false)}>
                                <X className="w-4 h-4" />
                            </Button>
                        </div>
                    </CardHeader>

                    <ScrollArea className="flex-1 p-4" ref={scrollRef}>
                        <div className="space-y-4">
                            {messages.map((msg, i) => (
                                <div key={i} className={`flex ${msg.role === "user" ? "justify-end" : "justify-start"}`}>
                                    <div className={`
                                max-w-[80%] rounded-2xl px-4 py-2 text-sm
                                ${msg.role === "user"
                                            ? "bg-cyan-600 text-white rounded-br-none"
                                            : "bg-zinc-800 text-zinc-200 rounded-bl-none border border-zinc-700"}
                            `}>
                                        {msg.content}
                                    </div>
                                </div>
                            ))}
                            {isLoading && (
                                <div className="flex justify-start">
                                    <div className="bg-zinc-800 p-3 rounded-2xl rounded-bl-none border border-zinc-700 flex gap-1">
                                        <span className="w-2 h-2 bg-zinc-500 rounded-full animate-bounce [animation-delay:-0.3s]"></span>
                                        <span className="w-2 h-2 bg-zinc-500 rounded-full animate-bounce [animation-delay:-0.15s]"></span>
                                        <span className="w-2 h-2 bg-zinc-500 rounded-full animate-bounce"></span>
                                    </div>
                                </div>
                            )}
                        </div>
                    </ScrollArea>

                    <CardFooter className="p-3 border-t border-zinc-800 bg-zinc-900/50">
                        <form
                            onSubmit={(e) => { e.preventDefault(); handleSendMessage(); }}
                            className="flex w-full gap-2"
                        >
                            <Input
                                placeholder="Type your question..."
                                className="bg-zinc-950 border-zinc-700 focus:border-cyan-500 text-white"
                                value={inputValue}
                                onChange={(e) => setInputValue(e.target.value)}
                            />
                            <Button type="submit" size="icon" className="bg-cyan-600 hover:bg-cyan-500" disabled={isLoading}>
                                <Send className="w-4 h-4" />
                            </Button>
                        </form>
                    </CardFooter>
                </Card>
            )}
        </div>
    );
}

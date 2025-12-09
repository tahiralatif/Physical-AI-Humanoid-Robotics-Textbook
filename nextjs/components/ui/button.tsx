import * as React from "react"
import { cva, type VariantProps } from "class-variance-authority"
import { cn } from "@/lib/utils"

const buttonVariants = cva(
    "inline-flex items-center justify-center gap-2 whitespace-nowrap rounded-xl text-sm font-semibold ring-offset-background transition-all focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring focus-visible:ring-offset-2 disabled:pointer-events-none disabled:opacity-50 uppercase tracking-wide",
    {
        variants: {
            variant: {
                primary:
                    "bg-gradient-primary text-white shadow-lg hover:shadow-xl hover:scale-105 hover:glow-purple active:scale-95",
                secondary:
                    "bg-gradient-secondary text-white shadow-lg hover:shadow-xl hover:scale-105 hover:glow-pink active:scale-95",
                outline:
                    "border-2 border-primary-start bg-transparent hover:bg-gradient-primary hover:text-white hover:scale-105 active:scale-95",
                ghost:
                    "hover:bg-accent/10 hover:text-accent-foreground",
            },
            size: {
                default: "h-12 px-6 py-3",
                sm: "h-9 rounded-lg px-4 text-xs",
                lg: "h-14 rounded-xl px-8 text-base",
                icon: "h-10 w-10",
            },
        },
        defaultVariants: {
            variant: "primary",
            size: "default",
        },
    }
)

export interface ButtonProps
    extends React.ButtonHTMLAttributes<HTMLButtonElement>,
    VariantProps<typeof buttonVariants> {
    asChild?: boolean
}

const Button = React.forwardRef<HTMLButtonElement, ButtonProps>(
    ({ className, variant, size, ...props }, ref) => {
        return (
            <button
                className={cn(buttonVariants({ variant, size, className }))}
                ref={ref}
                {...props}
            />
        )
    }
)
Button.displayName = "Button"

export { Button, buttonVariants }

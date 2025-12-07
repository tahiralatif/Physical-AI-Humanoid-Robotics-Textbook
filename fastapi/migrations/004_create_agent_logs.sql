-- Migration 004: Create agent_call_logs table
-- Physical AI Textbook Database Schema

-- Create agent_call_logs table
CREATE TABLE agent_call_logs (
    log_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    agent_name VARCHAR(100) NOT NULL,
    task_description TEXT NOT NULL,
    mcp_servers_used TEXT[],
    outcome VARCHAR(20) CHECK (outcome IN ('success', 'failure', 'partial')),
    execution_time_ms INT,
    created_at TIMESTAMP DEFAULT NOW()
);

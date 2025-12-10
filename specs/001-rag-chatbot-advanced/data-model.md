# Data Model: RAG Chatbot Advanced Features

## Entity: User

**Description**: Represents a student with background information for personalization

**Fields**:
- `id` (UUID/Integer): Primary key, unique identifier
- `email` (String): User's email address, unique, validated format
- `name` (String): User's full name
- `password_hash` (String): Bcrypt hash of user's password
- `software_background` (Enum): beginner | intermediate | advanced
- `hardware_background` (Enum): none | basic | advanced
- `created_at` (DateTime): Timestamp of account creation
- `updated_at` (DateTime): Timestamp of last update

**Validation rules**:
- Email: Must be valid email format
- Password: Minimum 8 characters (validation at signup)
- Software_background: Required field, one of the enum values
- Hardware_background: Required field, one of the enum values

**Relationships**:
- One-to-many with PersonalizedContent (user_id)
- One-to-many with Translation (user_id)
- One-to-many with Conversation (user_id, optional for authenticated conversations)

## Entity: PersonalizedContent

**Description**: Stores personalized chapter content generated for specific users

**Fields**:
- `id` (UUID/Integer): Primary key, unique identifier
- `user_id` (UUID/Integer): Foreign key to User
- `chapter_id` (String): Identifier for the original chapter
- `content` (Text): The personalized content
- `created_at` (DateTime): Timestamp of personalization
- `updated_at` (DateTime): Timestamp of last access/refresh

**Validation rules**:
- user_id: Must reference valid user
- chapter_id: Required field
- content: Required field

**Relationships**:
- Many-to-one with User (user_id)
- One-to-many with User (via user_id)

## Entity: Translation

**Description**: Stores translated chapter content in specific languages

**Fields**:
- `id` (UUID/Integer): Primary key, unique identifier
- `chapter_id` (String): Identifier for the original chapter
- `language` (String): Language code (e.g., 'ur' for Urdu)
- `content` (Text): The translated content
- `created_at` (DateTime): Timestamp of translation
- `updated_at` (DateTime): Timestamp of last access/refresh

**Validation rules**:
- chapter_id: Required field
- language: Required field, valid language code
- content: Required field

**Relationships**:
- One-to-many with chapters (conceptually, chapter_id references original content)

## Entity: Conversation (from Part 1, enhanced)

**Description**: Represents a chat session between user and AI

**Fields**:
- `id` (UUID/Integer): Primary key, unique identifier
- `session_id` (String): Unique session identifier
- `user_id` (UUID/Integer): Foreign key to User (optional, for authenticated sessions)
- `created_at` (DateTime): Timestamp of conversation start
- `updated_at` (DateTime): Timestamp of last message

**Validation rules**:
- session_id: Required field, unique
- user_id: Optional, references valid user if authenticated

**Relationships**:
- One-to-many with Message (conversation_id)
- Many-to-one with User (user_id, optional)

## Entity: Message (from Part 1)

**Description**: Represents a single message in a conversation

**Fields**:
- `id` (UUID/Integer): Primary key, unique identifier
- `conversation_id` (UUID/Integer): Foreign key to Conversation
- `role` (Enum): user | assistant
- `content` (Text): The message content
- `sources` (JSON/Text): List of content sources used in response
- `timestamp` (DateTime): When the message was created

**Validation rules**:
- conversation_id: Required field, references valid conversation
- role: Required field, one of enum values
- content: Required field

**Relationships**:
- Many-to-one with Conversation (conversation_id)
- One-to-many with Conversation (via conversation_id)

## Entity: ContentChunk (from Part 1, updated)

**Description**: Represents a chunk of textbook content stored in vector database

**Fields**:
- `chunk_id` (String): Unique identifier for the chunk
- `module` (String): Which module/chapter the content belongs to
- `week` (String): Which week of the curriculum
- `chapter_title` (String): Title of the specific chapter
- `content` (Text): The actual text content
- `url` (String): URL where the content can be found in the textbook
- `embedding` (Vector): 768-dimensional vector representation (Gemini embedding-001)

**Validation rules**:
- chunk_id: Required field, unique
- content: Required field
- embedding: Required field, 768 dimensions

**Relationships**:
- Stored in Qdrant vector database, referenced by the RAG service

## State Transitions

### User Authentication State
- `Unauthenticated` → `Authenticated` (via successful signin)
- `Authenticated` → `Unauthenticated` (via signout or token expiry)

### Content Personalization State
- `Original` → `Personalized` (when user clicks personalize button)
- `Personalized` → `Original` (when user clicks reset button)

### Content Translation State
- `English` → `Urdu` (when user clicks translate button)
- `Urdu` → `English` (when user clicks toggle/translate back button)

## Database Schema Considerations

### Neon Postgres Schema

```sql
-- Users table
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    name VARCHAR(255) NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    software_background VARCHAR(20) NOT NULL CHECK (software_background IN ('beginner', 'intermediate', 'advanced')),
    hardware_background VARCHAR(20) NOT NULL CHECK (hardware_background IN ('none', 'basic', 'advanced')),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Personalized content table
CREATE TABLE personalized_content (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    chapter_id VARCHAR(255) NOT NULL,
    content TEXT NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Translations table
CREATE TABLE translations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter_id VARCHAR(255) NOT NULL,
    language VARCHAR(10) NOT NULL,
    content TEXT NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Conversations table
CREATE TABLE conversations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id VARCHAR(255) UNIQUE NOT NULL,
    user_id UUID REFERENCES users(id) ON DELETE SET NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Messages table
CREATE TABLE messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    conversation_id UUID NOT NULL REFERENCES conversations(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    sources JSONB,
    timestamp TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);
```

### Indexes for Performance
- Index on users.email for fast authentication lookups
- Index on personalized_content.user_id and chapter_id for fast personalization retrieval
- Index on translations.chapter_id and language for fast translation retrieval
- Index on conversations.session_id and user_id for fast conversation lookups
- Index on messages.conversation_id for fast message retrieval